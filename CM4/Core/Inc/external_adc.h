/*
 * external_adc.h
 *
 *  Created on: Nov 22, 2020
 *      Author: Michal-Dell
 */

#ifndef INC_EXTERNAL_ADC_H_
#define INC_EXTERNAL_ADC_H_

#include "stm32h7xx.h"
#include "main.h"


/////// ADC ADS8691  ////////////
//INPUT COMMAND WORD AND REGISTER WRITE OPERATION
#define ADC_CLEAR_HWORD 0X60
#define ADC_READ_HWORD 0xCE	// 0xC8
#define ADC_READ 0x48
#define ADC_WRITE 0xD0
#define ADC_WRITE_MS 0xD2
#define ADC_WRITE_LS 0xD4
#define ADC_SET_HWORD 0xD8

//DEVICE CONFIGURATION AND REGISTER MAPS
#define ADC_DEVICE_ID_REG 0x02 //Device ID register
#define ADC_RST_PWRCTL_REG 0x04 //Reset and power control register
#define ADC_SDI_CTL_REG 0x08 //SDI data input control register
#define ADC_SDO_CTL_REG 0x0C //SDO-x data input control register
#define ADC_DATAOUT_CTL_REG 0x10 //Ouput data control register
#define ADC_RANGE_SEL_REG 0x14 //Input range selection control register
#define ADC_ALARM_REG 0x20 //ALARM output register
#define ADC_ALARM_H_TH_REG 0x24 //ALARM high threshold and hysteresis register
#define ADC_ALARM_L_TH_REG 0x28 //ALARM low threshold register

//INPUT RANGE SELECTION CONTROL REGISTER VALUES
#define ADC_RANGE_BIDIR_3VREF_INT_REF 0x0 //+- 3xVref, Internal reference
#define ADC_RANGE_BIDIR_25VREF_INT_REF 0x1 //+- 3xVref, Internal reference
#define ADC_RANGE_BIDIR_15VREF_INT_REF 0x2 //+- 3xVref, Internal reference
#define ADC_RANGE_BIDIR_125REF_INT_REF 0x3 //+- 3xVref, Internal reference
#define ADC_RANGE_BIDIR_0625VREF_INT_REF 0x4 //+- 3xVref, Internal reference
#define ADC_RANGE_UNIDIR_3VREF_INT_REF 0x8 //+- 3xVref, Internal reference
#define ADC_RANGE_UNIDIR_25VREF_INT_REF 0x9 //+- 3xVref, Internal reference
#define ADC_RANGE_UNIDIR_15VREF_INT_REF 0xA //+- 3xVref, Internal reference
#define ADC_RANGE_UNIDIR_125REF_INT_REF 0xB //+- 3xVref, Internal reference

// pins for ADC control


#define ADC_RESET_PIN ADC_RST_Pin
#define ADC_RESET_PORT ADC_RST_GPIO_Port
#define ADC_RSV_PIN	ADC_RVS_Pin
#define ADC_RSV_PORT ADC_RVS_GPIO_Port
//#define ADC_ALARM_PIN Extra_GPIO_Pin
//#define ADC_ALARM_PORT Extra_GPIO_GPIO_Port
#define ADC_CONV_PIN ADC_CONV_Pin
#define ADC_CONV_PORT ADC_CONV_GPIO_Port



// define lower and upper limit for switching + change ratio (linear regresion --> derivation --> koef)
#define RANGE_UPPER_LIMIT_NA			2.
#define RANGE_LOWER_LIMIT_NA			0.001
#define RANGE_UPPER_CHANGE_RATIO_NA		0.3
#define RANGE_LOWER_CHANGE_RATIO_NA		0.3
#define RANGE_UPPER_TOTAL_LIMIT_NA 		4.75
#define RANGE_LOWER_TOTAL_LIMIT_NA		0.001

#define RANGE_UPPER_LIMIT_UA			RANGE_UPPER_LIMIT_NA
#define RANGE_LOWER_LIMIT_UA			RANGE_LOWER_LIMIT_NA
#define RANGE_UPPER_CHANGE_RATIO_UA		RANGE_UPPER_CHANGE_RATIO_NA
#define RANGE_LOWER_CHANGE_RATIO_UA		RANGE_LOWER_CHANGE_RATIO_NA
#define RANGE_UPPER_TOTAL_LIMIT_UA 		RANGE_UPPER_TOTAL_LIMIT_NA
#define RANGE_LOWER_TOTAL_LIMIT_UA		RANGE_LOWER_TOTAL_LIMIT_NA

#define RANGE_UPPER_LIMIT_MA			RANGE_UPPER_LIMIT_NA
#define RANGE_LOWER_LIMIT_MA			RANGE_LOWER_LIMIT_NA
#define RANGE_UPPER_CHANGE_RATIO_MA		RANGE_UPPER_CHANGE_RATIO_NA
#define RANGE_LOWER_CHANGE_RATIO_MA		RANGE_LOWER_CHANGE_RATIO_NA
#define RANGE_UPPER_TOTAL_LIMIT_MA 		RANGE_UPPER_TOTAL_LIMIT_NA
#define RANGE_LOWER_TOTAL_LIMIT_MA		RANGE_LOWER_TOTAL_LIMIT_NA


// ADC parameters
// OLD config (default alfter restart)
//#define ADC_SCALE 262144		// 18 bit ADC
//#define ADC_REF_VALUE 4096		// ref voltage 4,096 V
//#define ADC_PGA 3				// PGA gain set to +- 3xVref

#define ADC_SCALE 262144		// 18 bit ADC
#define ADC_REF_VALUE 5000		// ref voltage 5,000 V
#define ADC_PGA 1.25				// PGA gain set to +- 1.25xVref
#define ADC_BIDIRECTIONAL 0			// 0 - unidirectional, 1 - bidirectional
#define ADC_DIRECTION 1		// 1 - unidirectional, 2 - bidirectional
//#define ADC_RESOLUTION ((ADC_REF_VALUE*ADC_PGA*ADC_DIRECTION)/ADC_SCALE)
//#define ADC_RESOLUTION 0.01953125‬

#define MS_BYTE(x) ((UINT8)(x >> 8))

#define LS_BYTE(x) ((UINT8)(x & 0xff))


/// private variables definitions

// write data to specific register
uint8_t adc_write_data(uint8_t command, uint8_t regAdrr, uint8_t dataValueMS, uint8_t dataValueLS);

// sequence to config ADC after reset or power up
uint8_t adc_config();

// soft reset ADC
void adc_reset();

// NEW ADC TI ADS8910 18bit, 1Msps, differencial
double adc_sample();

// OLD ADC TI ADS8691 18bit, 1Msps, with buffer, single ended
double adc_sample_ads8691();



#endif /* INC_EXTERNAL_ADC_H_ */
