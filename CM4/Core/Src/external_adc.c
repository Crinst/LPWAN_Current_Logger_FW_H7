/*
 * external_adc.c
 *
 *  Created on: Nov 22, 2020
 *      Author: Michal-Dell
 */

// private variables definitions

#include "external_adc.h"

// ADC SPI buffers
uint8_t spiDataTx[4];
uint8_t spiDataRx[4];


// STM HAL return value
HAL_SPI_StateTypeDef returnValue;


// ADC regarding values
//double ADC_RESOLUTION = 0;
double measuredValue = 0;
double previousMeasuredValue = 0;
double measuredValueCurrent = 0;

double ADC_RESOLUTION = 0.01953125; //0.038146973;


// extern
extern uint8_t isWaitingForData;
extern SPI_HandleTypeDef hspi2;



uint8_t adc_write_data(uint8_t command, uint8_t regAdrr, uint8_t dataValueMS, uint8_t dataValueLS){


	isWaitingForData = 1;		// transfer compete flag

	uint8_t bufferTx [4];		// spi Tx buffer 4 bytes = 1 word
	uint8_t bufferRx [4];		// spi Rx buffer 4 bytes = 1 word

	bufferTx [0] = command;				// write 8 bit command word
	bufferTx [1] = regAdrr;				// write 8 bit register address
	bufferTx [2] = dataValueMS;			// 16 bit register value --> 8 MSB bit into register
	bufferTx [3] = dataValueLS;			// 16 bit register value --> 8 LSB bit into register

	// to GPI LOW
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);

	microDelay(50);

	HAL_SPI_TransmitReceive_IT(&hspi2, bufferTx, bufferRx, 4);
	//HAL_SPI_TransmitReceive_DMA(&hspi2, bufferTx, bufferRx, 4);

	while (isWaitingForData > 0);

	microDelay(50);
	// to GPI HIGH
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

	microDelay(50);

	// to GPI LOW
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_RESET);
	microDelay(50);

	HAL_SPI_TransmitReceive_IT(&hspi2, bufferTx, bufferRx, 4);
	//HAL_SPI_TransmitReceive_DMA(&hspi2, bufferTx, bufferRx, 4);

	while (isWaitingForData > 0);

	microDelay(50);
	// to GPI HIGH
	HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, GPIO_PIN_SET);

	microDelay(50);


}

// sequence to config ADC after reset or power up
uint8_t adc_config() {

	isWaitingForData = 1;

	// SETTING ADC PGA AND WORKING RANGE
	adc_write_data(ADC_WRITE, ADC_RANGE_SEL_REG, 0x0, ADC_RANGE_UNIDIR_125REF_INT_REF);


}

void adc_reset(){

	// to GPIO LOW
	HAL_GPIO_WritePin(ADC_RESET_PORT, ADC_RESET_PIN, GPIO_PIN_RESET);
	microDelay(5);
	// to GPIO HIGH
	HAL_GPIO_WritePin(ADC_RESET_PORT, ADC_RESET_PIN, GPIO_PIN_SET);

	HAL_Delay(25);

	send_uart("ADC resetted\n");

}


// NEW ADC TI ADS8910 18bit, 1Msps, differencial
double adc_sample() {

	isWaitingForData = 1;
	uint32_t receivedValue = 0;
	//isAdcDone = 0;

	// dummy data to clock data out of ADC = No operation command
	spiDataTx[0] = 0x00;	// 0000 1000
	spiDataTx[1] = 0x00;	// 0000 0000
	spiDataTx[2] = 0x00;	// 0000 0000
	//spiDataTx[3] = 0x00;	// 0000 0000

	// starting conversion
	//HAL_GPIO_WritePin(ADC_CONV_PORT, ADC_CONV_PIN, GPIO_PIN_SET);
	// CONV port B pin 5
	// SET HIGH
	GPIOB->ODR |= (1<<5);
	//microDelay(1);

	//HAL_GPIO_WritePin(ADC_CONV_PORT, ADC_CONV_PIN, GPIO_PIN_RESET);
	// CONV port B pin 5
	// SET LOW
	GPIOB->ODR &= ~(1<<5);

	// RVS port D pin 5 check for transition LOW-->HIGH
	//while( (GPIOD->IDR & 0x20) != 1);
	//while(HAL_GPIO_ReadPin(ADC_RSV_PORT, ADC_RSV_PIN) != GPIO_PIN_SET);
	//while(isAdcDone == 0);
	HAL_Delay(10);

	// acquiring measured data
	// to CS LOW
	//HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	// CS port D pin 6
	// SET LOW
	GPIOD->ODR &= ~(1<<6);

	//HAL_SPI_TransmitReceive(&hspi2, spiDataTx, spiDataRx, 4, 1000);
	HAL_SPI_TransmitReceive_DMA(&hspi2, spiDataTx, spiDataRx, 3);
	//HAL_SPI_TransmitReceive_IT(&hspi2, spiDataTx, spiDataRx, 4);
	//HAL_SPI_Receive_DMA(&hspi2, spiDataRx, 4);

	while (isWaitingForData > 0);
	//while (HAL_DMA_GetState(&hdma_spi1_rx) == HAL_DMA_STATE_RESET);

	// to CS HIGH
 	//HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
 	// CS port D pin 6
 	// SET HIGH
 	GPIOD->ODR |= (1<<6);

	//shifting received byte data into final value
 	// ADS8690  // ADS8910
	receivedValue = (spiDataRx[2] >> 6) | (spiDataRx[1] << 2)	| (spiDataRx[0] << 10);

	measuredValue = receivedValue;

	if(receivedValue <= 131071){
		measuredValue = receivedValue;
		measuredValue = (measuredValue * ADC_RESOLUTION);
		// adding bidirectional offset value
		//measuredValue = measuredValue - (ADC_REF_VALUE * ADC_PGA*ADC_BIDIRECTIONAL);
		// shifting from mV to V value
		measuredValue /= 1000;
		//measuredValue *=2.186;
		// adding measured DC offset value
		//measuredValue -= settings.lastOffsetValue;
	}
	else{
		measuredValue = receivedValue - 0x1FFFF;

		measuredValue = measuredValue * ADC_RESOLUTION; //(-ADC_REF_VALUE + temp);
		measuredValue = measuredValue - ADC_REF_VALUE;
		// shifting from mV to V value
		measuredValue /= 1000;
		// adding measured DC offset value
		//measuredValue -= settings.lastOffsetValue;

	}

/*
	// change range filling values
	//previousValues[SAMPLES - 1] = measuredValue;
	previousValues[currentValuePosition % SAMPLES] = measuredValue;
	previousValuesRange[currentValuePosition % SAMPLES]= currentRange;
	currentValuePosition++;
*/
	return measuredValue;

}


// OLD ADC TI ADS8691 18bit, 1Msps, with buffer, single ended
double adc_sample_ads8691() {

	isWaitingForData = 1;
	uint32_t receivedValue = 0;
	//isAdcDone = 0;

	// dummy data to clock data out of ADC = No operation command
	spiDataTx[0] = 0x00;	// 0000 1000
	spiDataTx[1] = 0x00;	// 0000 0000
	spiDataTx[2] = 0x00;	// 0000 0000
	//spiDataTx[3] = 0x00;	// 0000 0000

	// acquiring measured data
	// to CS LOW
	//HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	// CS port D pin 6
	// SET LOW
	GPIOB->ODR &= ~(1<<11);

	// to CS HIGH
	//HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
	// CS port D pin 6
	// SET HIGH
	GPIOB->ODR |= (1<<11);

	// RVS port D pin 5 check for transition LOW-->HIGH
	//while( (GPIOD->IDR & 0x20) != 1);
	//while(HAL_GPIO_ReadPin(ADC_RSV_PORT, ADC_RSV_PIN) != GPIO_PIN_SET);
	//while(isAdcDone == 0);
	HAL_Delay(10);

	// acquiring measured data
	// to CS LOW
	//HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
	// CS port D pin 6
	// SET LOW
	GPIOB->ODR &= ~(1<<11);

	//HAL_SPI_TransmitReceive(&hspi2, spiDataTx, spiDataRx, 4, 1000);
	HAL_SPI_TransmitReceive_DMA(&hspi2, spiDataTx, spiDataRx, 3);
	//HAL_SPI_TransmitReceive_IT(&hspi2, spiDataTx, spiDataRx, 4);
	//HAL_SPI_Receive_DMA(&hspi2, spiDataRx, 4);

	while (isWaitingForData > 0);
	//while (HAL_DMA_GetState(&hdma_spi1_rx) == HAL_DMA_STATE_RESET);

	// to CS HIGH
 	//HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
 	// CS port D pin 6
 	// SET HIGH
 	GPIOB->ODR |= (1<<11);

	//shifting received byte data into final value
 	// ADS8690  // ADS8910
	receivedValue = (spiDataRx[2] >> 6) | (spiDataRx[1] << 2)	| (spiDataRx[0] << 10);

	measuredValue = receivedValue;
	//measuredValue = (measuredValue * (0.09375));
	measuredValue = (measuredValue * ADC_RESOLUTION);
	//measuredValue = ( measuredValue * ( (ADC_REF_VALUE*ADC_PGA*2) / ADC_SCALE ) );

	// adding bidirectional offset value
	measuredValue = measuredValue - (ADC_REF_VALUE * ADC_PGA*ADC_BIDIRECTIONAL);
	// shifting from mV to V value
	measuredValue /= 1000;

	/*
	// adding measured DC offset value
	measuredValue -= settings.lastOffsetValue;

	// change range filling values
	//previousValues[SAMPLES - 1] = measuredValue;
	previousValues[currentValuePosition % SAMPLES] = measuredValue;
	previousValuesRange[currentValuePosition % SAMPLES]= currentRange;
	currentValuePosition++;
	*/
	return measuredValue;

}
