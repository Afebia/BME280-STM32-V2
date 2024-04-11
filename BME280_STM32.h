/*
 * BME280_STM32.h
 *
 *  Created on: Apr 3, 2024
 *      Author: berat
 */

#ifndef INC_BME280_STM32_H_
#define INC_BME280_STM32_H_

#include <stdint.h>
#include <stdio.h>
#include "stm32f4xx_hal.h"
#include <math.h>
#include <stdbool.h>

extern I2C_HandleTypeDef hi2c1;


#define BME280_ADDR (0x76 << 1) //0xEC  I2C Address

#define CHIP_ID_BME 0x60

//RAW DATA
#define HUM_LSB_P 0xFE
#define HUM_MSB_P 0xFD
#define TEMP_XLSB_P 0xFC
#define TEMP_LSB_P 0xFB
#define TEMP_MSB_P 0xFA
#define PRESS_XLSB_P 0xF9
#define PRESS_LSB_P 0xF8
#define PRESS_MSB_P 0xF7

#define CONFIG_P 0xF5    //BME280 CONFIG register
#define CTRL_MEAS_P 0xF4 //BME280 CTRL_Meas register
#define CHIP_ID_P 0xD0   //BME280 CHIP_ID register
#define CTRL_HUM_P 0xF2  //BME280 CTRL_HUM register
#define RESET_P 0xE0     //BME280 RESET register
#define BME280_ID_P 0xD0 //CHIP ID, which is 0x60

#define CALIB_DATA1_P 0x88  // Calibration register, Sıcaklık ve Basınç için Compensation Values
#define CALIB_DATA2_P 0xE1  // Calibration register, Nem için Compensation Values

typedef enum{
	SKIPPED = 0,
	OVERSAMPLING_1,
	OVERSAMPLING_2,
	OVERSAMPLING_4,
	OVERSAMPLING_8,
	OVERSAMPLING_16
}oversampling_t;

typedef enum{ //ACCEL RANGES
	SLEEP =0x00,
	FORCED=0x01,
	NORMAL=0x03
}sensor_mode_t;

typedef enum{
	 T_SB_0_5 = 0x00,
	 T_SB_62_5,
	 T_SB_125,
	 T_SB_250,
	 T_SB_500,
	 T_SB_1000,
	 T_SB_10,
	 T_SB_20
}standby_t;

typedef enum{
	 FILTER_OFF = 0x00,
	 FILTER_2,
 	 FILTER_4,
	 FILTER_8,
	 FILTER_16
}filter_t;

typedef struct{

	int32_t tempr;
	int32_t pressr;
	uint16_t humr;
}raw_data_t;

void Calibdata_BME280(void);  //Compensation Value Okuma
void RawdataBME280(void);    //Raw Data Okuma
void BME280Init(oversampling_t oversamplingT , oversampling_t  oversamplingP,
		oversampling_t oversamplingH,sensor_mode_t mode,standby_t t_sb,filter_t filter);  //İnitizaliton Fonksiyonu
void BME280Calculation(void);  //Sıcaklık, Basınç ve Nem Değerlerini Hesaplama

uint32_t BME280_measure_Hum(int32_t adc_H);
uint32_t BME280_measure_Press(int32_t adc_P);
int32_t BME280_measure_Temp(int32_t adc_T);

void Reset_BME280(void);

#endif /* INC_BME280_STM32_H_ */
