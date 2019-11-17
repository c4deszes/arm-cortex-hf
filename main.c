/**
 * \file
 *
 * \brief Frequency Response Analyzer
 */

#include <atmel_start.h>
#include "hpl_dma.h"
#include "limits.h"
#include "semphr.h"
#include "timers.h"
#include <math.h>
#include "fastmath.h"
#include "utils.h"

/* RTOS Tasks Constants */
#define TASK_DEFAULT_STACK_SIZE (4096 / sizeof(portSTACK_TYPE))
#define MEASURE_PRIORITY (tskIDLE_PRIORITY + 1)
#define DMA_PRIORITY configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY+1

/* The priority of the peripheral should be between the low and high interrupt priority set by chosen RTOS,
 * Otherwise, some of the RTOS APIs may fail to work inside interrupts
 * In case of FreeRTOS,the Lowest Interrupt priority is set by configLIBRARY_LOWEST_INTERRUPT_PRIORITY and
 * Maximum interrupt priority by configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY, So Interrupt Priority of the peripheral
 * should be between configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY and configLIBRARY_LOWEST_INTERRUPT_PRIORITY
 */
#define PERIPHERAL_INTERRUPT_PRIORITY (configLIBRARY_LOWEST_INTERRUPT_PRIORITY - 1)

/* RTOS Tasks Handle */
static TaskHandle_t measureTask;
#define SAMPLE_SIZE 512

/** \brief  The amount of samples generated each second
	Used for frequency generation 
*/
#define SAMPLE_RATE 1000000UL

/** \brief Output signal amplitude 

*/
#define WAVEFORM_AMPLITUDE 1800

/** \brief Generates a sine wave of a given frequency into a buffer

@param freq Frequency of the desired sine wave in terms of seconds 1/s
@param buffer The target array in which the samples should be stored
@param length The size of the target buffer

*/
void sine_wave(float freq, uint16_t* buffer, uint16_t length) {
	int i = 0;
	while(i < length) {
		buffer[i] = (uint16_t)(2047 + (WAVEFORM_AMPLITUDE * sinf(2 * M_PI * freq / SAMPLE_RATE * i)));
		i++;
	}
}

#define WAVEFORM_LENGTH 512

/** \brief The length of one period at the current frequency */
uint32_t period;

/** \brief The output waveform buffer */
uint16_t dut_out[WAVEFORM_LENGTH];

/** \brief DAC is driven by the 4th DMAC channel */
#define DMAC_CHANNEL_DAC 4U

/** \brief Configures the DAC output and the DMA channel for it

Enables the DAC output
Enables event output for the Empty event, this is going to trigger the ADC0/1 conversion
Enables interrupts for the Empty event, this is going to trigger the DMA beat transfer

Configures the DMA channel source and destination addresses to move data from 'dut_out' to DAC0 Data
The transaction is reset in the interrupt handler
	
*/
void config_wave_out() {
	//Enable DAC peripheral clock
	hri_mclk_set_APBDMASK_DAC_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, DAC_GCLK_ID, CONF_GCLK_DAC_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	
	//Initialize the DAC with the configuration inside 'Config/hpl_dac_config.h'
	dac_os_init(&DAC_0, DAC);
	
	//Enable DAC output
	gpio_set_pin_direction(PA02, GPIO_DIRECTION_OFF);
	gpio_set_pin_function(PA02, PINMUX_PA02B_DAC_VOUT0);
	
	/* Configure DMAC to trigger DAC based on 
	 * Connect PA03(in SAME54 Xplained Pro EXT2) to VDD - because of
	 * ERRATUM */
	hri_dac_set_DACCTRL_ENABLE_bit(DAC, false);
	hri_dac_set_EVCTRL_EMPTYEO0_bit(DAC);			//Enable event control for the empty interrupt so we can trigger the ADC using this channel
	hri_dac_set_INTEN_EMPTY0_bit(DAC);
	NVIC_SetPriority(DAC_1_IRQn, PERIPHERAL_INTERRUPT_PRIORITY);
	NVIC_EnableIRQ(DAC_1_IRQn);						
	hri_dac_set_CTRLA_ENABLE_bit(DAC);				//Starts the DAC
	
	_dma_set_source_address(DMAC_CHANNEL_DAC, (const void *)dut_out);
	_dma_set_destination_address(DMAC_CHANNEL_DAC, (const void *)&DAC->DATA[0].reg);
	NVIC_SetPriority((IRQn_Type)DMAC_4_IRQn, DMA_PRIORITY);
	NVIC_EnableIRQ(DMAC_4_IRQn);
	hri_dmac_set_CHINTEN_TCMPL_bit(DMAC, DMAC_CHANNEL_DAC);
}

/** \brief Clears DAC Empty interrupt flag */
void DAC_1_Handler(void) {
	hri_dac_clear_INTFLAG_EMPTY0_bit(DAC);
}

/** \brief Resets the DMA transaction so the waveform is continous */
void DMAC_4_Handler(void) {
	hri_dmac_clear_CHINTFLAG_TCMPL_bit(DMAC, DMAC_CHANNEL_DAC);
	_dma_enable_transaction(DMAC_CHANNEL_DAC,true);
}

/** \brief Reference input buffer 1st half */
#define DMAC_CHANNEL_REF1 2U
/** \brief Reference input buffer 2nd half */
#define DMAC_CHANNEL_REF2 3U

/** \brief Reference input buffer */
uint16_t ref_in[SAMPLE_SIZE];

/** \brief Configures the reference signal input and the DMA channel for it

Enables ADC1, routes AIN0 to PB08
Enables Event input for the conversion start event, this is triggered by EVSYS CH0 which is connected to DAC Empty0

Configures the DMA channel source and destination addresses to move data from ADC1 Result to 'ref_in'
The transaction is double buffered by using 2 channels setup to different addresses and on completion
the channel triggers the other channel to start.

We could achieve the same result by using the next descriptor value and create a loop, but that doesn't
give us an interrupt signal.

*/
void config_ref_in() {
	
	hri_mclk_set_APBDMASK_ADC1_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, ADC1_GCLK_ID, CONF_GCLK_ADC1_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	
	gpio_set_pin_direction(PB08, GPIO_DIRECTION_OFF);
	gpio_set_pin_function(PB08, PINMUX_PB08B_ADC1_AIN0);
	
	//Enables interrupts for ADC1 result ready
	NVIC_SetPriority((IRQn_Type)ADC1_1_IRQn, PERIPHERAL_INTERRUPT_PRIORITY);
	NVIC_EnableIRQ(ADC1_1_IRQn);
	hri_adc_set_INTEN_RESRDY_bit(ADC1);			
	hri_adc_set_EVCTRL_STARTEI_bit(ADC1);		//Enable conversion start event
	hri_adc_set_CTRLA_ENABLE_bit(ADC1);			//Enables ADC1
	
	//REF1 and REF2 are triggered by the ADC1 Result Ready event
	_dma_set_source_address(DMAC_CHANNEL_REF1, (const void *)&ADC1->RESULT.reg);
	_dma_set_destination_address(DMAC_CHANNEL_REF1, (const void *)ref_in);
	_dma_set_data_amount(DMAC_CHANNEL_REF1, SAMPLE_SIZE/2);
	NVIC_SetPriority((IRQn_Type)DMAC_2_IRQn, DMA_PRIORITY);
	NVIC_EnableIRQ(DMAC_2_IRQn);
	hri_dmac_set_CHINTEN_TCMPL_bit(DMAC, DMAC_CHANNEL_REF1);
	
	_dma_set_source_address(DMAC_CHANNEL_REF2, (const void*)&ADC1->RESULT.reg);
	_dma_set_destination_address(DMAC_CHANNEL_REF2, (const void*)&(ref_in[SAMPLE_SIZE/2]));
	_dma_set_data_amount(DMAC_CHANNEL_REF2, SAMPLE_SIZE/2);
	NVIC_SetPriority((IRQn_Type)DMAC_3_IRQn, DMA_PRIORITY);
	NVIC_EnableIRQ(DMAC_3_IRQn);
	hri_dmac_set_CHINTEN_TCMPL_bit(DMAC, DMAC_CHANNEL_REF2);
}

/** \brief Resets the ADC1 Result Ready interrupt flag */
void ADC1_1_Handler(void) {
	hri_adc_clear_INTFLAG_RESRDY_bit(ADC1);
}

/** \brief REF1 completion triggers REF2 transfer to start */
void DMAC_2_Handler(void) {
	hri_dmac_clear_CHINTFLAG_TCMPL_bit(DMAC, DMAC_CHANNEL_REF1);
	_dma_enable_transaction(DMAC_CHANNEL_REF2, false);
}

/** \brief REF2 completion triggeres REF1 transfer to start */
void DMAC_3_Handler(void) {
	hri_dmac_clear_CHINTFLAG_TCMPL_bit(DMAC, DMAC_CHANNEL_REF2);
	_dma_enable_transaction(DMAC_CHANNEL_REF1, false);
}

/** \brief DUT input buffer 1st half */
#define DMAC_CHANNEL_DUT1 0U
/** \brief DUT input buffer 2nd half */
#define DMAC_CHANNEL_DUT2 1U

/** \brief DUT input buffer */
uint16_t dut_in[SAMPLE_SIZE];

/** \brief Configures the DUT signal input and the DMA channel for it

Enables ADC0, routes AIN6 to PA06
Enables Event input for the conversion start event, this is triggered by EVSYS CH0 which is connected to DAC Empty0

Configures the DMA channel source and destination addresses to move data from ADC0 Result to 'dut_in'
The transaction is double buffered by using 2 channels setup to different addresses and on completion
the channel triggers the other channel to start.

*/
void config_dut_in() {
	//Initialize ADC0 also known as DUT input
	hri_mclk_set_APBDMASK_ADC0_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, ADC0_GCLK_ID, CONF_GCLK_ADC0_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	
	//Routing AIN6 to Pin PA06
	gpio_set_pin_direction(PA06, GPIO_DIRECTION_OFF);
	gpio_set_pin_function(PA06, PINMUX_PA06B_ADC0_AIN6);
	
	NVIC_SetPriority((IRQn_Type)ADC0_1_IRQn, PERIPHERAL_INTERRUPT_PRIORITY);
	NVIC_EnableIRQ(ADC0_1_IRQn);				//Enable result ready interrupt
	hri_adc_set_INTEN_RESRDY_bit(ADC0);			
	hri_adc_set_EVCTRL_STARTEI_bit(ADC0);		//Enable event control conversion start bit
	hri_adc_set_CTRLA_ENABLE_bit(ADC0);			//Enable the ADC
	
	//DUT1 and DUT2 are triggered by the ADC0 Result Ready event
	_dma_set_source_address(DMAC_CHANNEL_DUT1, (const void *)&ADC0->RESULT.reg);
	_dma_set_destination_address(DMAC_CHANNEL_DUT1, (const void *)dut_in);
	_dma_set_data_amount(DMAC_CHANNEL_DUT1, SAMPLE_SIZE/2);
	NVIC_SetPriority((IRQn_Type)DMAC_0_IRQn, DMA_PRIORITY);
	NVIC_EnableIRQ(DMAC_0_IRQn);
	hri_dmac_set_CHINTEN_TCMPL_bit(DMAC, DMAC_CHANNEL_DUT1);
	
	_dma_set_source_address(DMAC_CHANNEL_DUT2, (const void *)&ADC0->RESULT.reg);
	_dma_set_destination_address(DMAC_CHANNEL_DUT2, (const void *)&(dut_in[SAMPLE_SIZE/2]));
	_dma_set_data_amount(DMAC_CHANNEL_DUT2, SAMPLE_SIZE/2);
	NVIC_SetPriority((IRQn_Type)DMAC_1_IRQn, DMA_PRIORITY);
	NVIC_EnableIRQ(DMAC_1_IRQn);
	hri_dmac_set_CHINTEN_TCMPL_bit(DMAC, DMAC_CHANNEL_DUT2);
}

/** \brief Clears ADC0 Result ready interrupt flag */
void ADC0_1_Handler(void) {
	hri_adc_clear_INTFLAG_RESRDY_bit(ADC0);
}

/** \brief Measurement index
The index where the sample starts, this is used to signal RTOS which half of the double buffer it should process 
*/
uint16_t measure_start;

/** \brief Measurement index
The index where the sample ends, this is used to signal RTOS which half of the double buffer it should process
*/
uint16_t measure_end;

/** \brief Binary semaphore used to trigger the measurement task */
SemaphoreHandle_t sync_lock = NULL;

/** \brief On filling the first half of the buffer it triggers the measurement task and the second half transaction */
void DMAC_0_Handler(void) {
	hri_dmac_clear_CHINTFLAG_TCMPL_bit(DMAC, DMAC_CHANNEL_DUT1);
	_dma_enable_transaction(DMAC_CHANNEL_DUT2, false);
	
	measure_start = 0;					//Processing the first half of the buffer
	measure_end = SAMPLE_SIZE / 2;
	
	//Triggering the RTOS task
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(sync_lock, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/** \brief On filling the second half of the buffer it triggers the measurement task and the first half transaction */
void DMAC_1_Handler(void) {
	hri_dmac_clear_CHINTFLAG_TCMPL_bit(DMAC, DMAC_CHANNEL_DUT2);
	_dma_enable_transaction(DMAC_CHANNEL_DUT1, false);
	
	measure_start = SAMPLE_SIZE / 2;	//Processing the second half of the buffer
	measure_end = SAMPLE_SIZE;
	
	//Triggering the RTOS task
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(sync_lock, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/** \brief Event System channel triggering the ADC conversions */
#define DAC_EVENT_CHANNEL 0

/** \brief Configures the Event System

Connects the DAC Empty0 event (when the DAC is ready for a new conversion) to the Event System Channel 0
Then connects the two user events, ADC0_START and ADC1_START to Channel 0

So when the DAC finishes we start both ADC0 and ADC1 conversions at the same time
 */
void config_evsys() {
	//Enables the clock of EVSYS
	hri_mclk_set_APBBMASK_EVSYS_bit(MCLK);
	hri_gclk_set_PCHCTRL_CHEN_bit(GCLK, EVSYS_GCLK_ID_0);
	
	//DAC Empty event is the generator
	//This triggers both the ADCs to start a conversion
	hri_evsys_set_CHANNEL_EDGSEL_bf(EVSYS, DAC_EVENT_CHANNEL, EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT_Val);
	hri_evsys_set_CHANNEL_PATH_bf(EVSYS, DAC_EVENT_CHANNEL, EVSYS_CHANNEL_PATH_ASYNCHRONOUS_Val);
	hri_evsys_set_CHANNEL_EVGEN_bf(EVSYS, DAC_EVENT_CHANNEL, EVSYS_ID_GEN_DAC_EMPTY_0);

	//Connects the user channels to EVSYS Channel 0
	hri_evsys_set_USER_CHANNEL_bf(EVSYS, EVSYS_ID_USER_ADC0_START, 0x01 + DAC_EVENT_CHANNEL);
	hri_evsys_set_USER_CHANNEL_bf(EVSYS, EVSYS_ID_USER_ADC1_START, 0x01 + DAC_EVENT_CHANNEL);
}

/** \brief Starts generating the waveform

This starts the self sustaining cycle of outputting the waveform and taking the samples

*/
void start_wavegen() {
	_dma_enable_transaction(DMAC_CHANNEL_DAC, true);
	_dma_enable_transaction(DMAC_CHANNEL_DUT1, false);
	_dma_enable_transaction(DMAC_CHANNEL_REF1, false);
}

/** \brief Sample minimum and maximum values, used to calculate the amplitude of the reference and incoming signal */
uint16_t dut_max, dut_min;
uint16_t ref_max, ref_min;

/** \brief The number of increments between the start and stop frequencies */
#define SWEEP_RESOLUTION 50

/** \brief The number of samples for each frequency */
#define SAMPLE_AMOUNT 20

/** \brief Sample counter for each frequency */
uint16_t sample_index;

/** \brief The first frequency to output.
This is limited by the output sample size, the larger it is the lower we can go.
The only way to completely eliminate this limit is by switching between DMA and interrupt driven DAC, 
for lower frequencies interrupting is less of an issue and the output can be calculated on the fly.

Other solutions would include keeping the same high resolution sample and using a timer and 
switching the step size of the DMAC transfer to have a better frequency range.
*/
#define START_FREQUENCY 1950

/** \brief The last frequency to output 
This is limited by sample rate and to achieve clean sine waves we have to keep this value low. 
*/
#define STOP_FREQUENCY 15600

/** \brief Index of the current frequency we are sampling */
uint16_t sweep_index;

/** \brief Amplitude values in dB for each frequency */
float amplitude[SWEEP_RESOLUTION];

/** \brief Task that is used to sweep across the frequency range, analyze samples 

*/
void measure_task(void) {
	
	//Creating the lock to synchronize the samples and the measurement
	sync_lock = xSemaphoreCreateBinary();
	
	//Initialize peripherals
	config_wave_out();
	config_ref_in();
	config_dut_in();
	config_evsys();
	
	//Initialize measurement
	dut_max = ref_max = 0;
	dut_min = ref_min = 0xFFFF;
	
	//Initialize first sine wave
	sweep_index = 0;
	sine_wave(START_FREQUENCY, dut_out, WAVEFORM_LENGTH);
	_dma_set_data_amount(DMAC_CHANNEL_DAC, SAMPLE_RATE / START_FREQUENCY);
	period = SAMPLE_RATE / START_FREQUENCY;
	
	//Enabling output
	start_wavegen();
	
	while(1) {
		//We wait for the interrupt
		xSemaphoreTake(sync_lock, 1);

		//Sample is ready, we measure the amplitude
		for(;measure_start < measure_end; measure_start++) {
			dut_max = max(dut_in[measure_start], dut_max);
			ref_max = max(ref_in[measure_start], ref_max);
			dut_min = min(dut_in[measure_start], dut_min);
			ref_min = min(ref_in[measure_start], ref_min);
		}
		sample_index++;
		
		//After a few samples
		if(sample_index >= SAMPLE_AMOUNT) {
			//We store the amplitude difference in dB into an array
			amplitude[sweep_index] = 20.0f * log10f((float)(dut_max-dut_min) / (float)(ref_max - ref_min));
			
			//Resetting the measurement
			sample_index = 0;
			dut_max = ref_max = 0;
			dut_min = ref_min = 0xFFFF;
			
			sweep_index++;
			//Next sweep
			if(sweep_index >= SWEEP_RESOLUTION) {
				sweep_index = 0;
			}
			vTaskDelay(1);
			
			//Generating next frequency in the current sweep
			float freq = (float)sweep_index / SWEEP_RESOLUTION * (STOP_FREQUENCY - START_FREQUENCY) + START_FREQUENCY;
			sine_wave(freq, dut_out, WAVEFORM_LENGTH);
			period = (uint32_t)(SAMPLE_RATE / freq);
			
			//Reconfigure the DMA
			hri_dmac_set_CHCTRLA_SWRST_bit(DMAC, DMAC_CHANNEL_DAC);
			_dma_set_source_address(DMAC_CHANNEL_DAC, (const void *)dut_out);
			_dma_set_destination_address(DMAC_CHANNEL_DAC, (const void *)&DAC->DATA[0].reg);
			_dma_set_data_amount(DMAC_CHANNEL_DAC, period - 5);
			hri_dmac_set_CHINTEN_TCMPL_bit(DMAC, DMAC_CHANNEL_DAC);
			_dma_enable_transaction(DMAC_CHANNEL_DAC,true);
		}
	}
}

/** \brief Initializes drivers and the measurement task, starts RTOS */
int main(void)
{
	/* Initializes MCU, drivers and middleware */
	atmel_start_init();

	/* Create the measurement task */
	xTaskCreate((TaskFunction_t)measure_task, "Measure_Task", TASK_DEFAULT_STACK_SIZE, NULL, MEASURE_PRIORITY, measureTask);

	/* Start the RTOS Scheduler */
	vTaskStartScheduler();

	while (1);
}
