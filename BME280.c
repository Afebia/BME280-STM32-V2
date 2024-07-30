/*
 * BME280.c
 *
 *  Created on: Mar 6, 2024
 *      Author: Berat Bayram
 */
#include "BME280_STM32.h"

/*
 * Compensation words definition
 */
unsigned short dig_T1,dig_P1, dig_H1, dig_H3;
signed short dig_T2, dig_T3,dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9, dig_H2,dig_H4, dig_H5, dig_H6;

int32_t t_fine;
/**
  * @brief  Software Reset to BME280
  *
  * @param  None
  *
  * @retval None
  */
void Reset_BME280(void){

	uint8_t data = 0xB6;
	HAL_I2C_Mem_Write(&bme_i2c, BME280_ADDR, RESET_REG_ADDR, 1, &data, 1, HAL_MAX_DELAY);
	HAL_Delay(500);

	//Checking for is reset process done
	uint8_t id;
	HAL_I2C_Mem_Read(&bme_i2c, BME280_ADDR, CHIP_ID_REG_ADDR, 1, &id, 1, HAL_MAX_DELAY);

	//If value of id register is not equal to BME280 chip id which is 0x60, wait until equal to each other
	while(id != CHIP_ID_BME){
		printf("BME280-> Undefined chip id\n");
		HAL_Delay(500);
	}
}

/**
  * @brief  Puts the BME280 into sleep mode
  *
  * @param  None
  *
  * @retval None
  */
HAL_StatusTypeDef BME280_SleepMode(void){
	uint8_t init =0;
	init = BME280_SLEEP_MODE;

	return HAL_I2C_Mem_Write(&bme_i2c, BME280_ADDR, CTRL_MEAS_REG_ADDR, 1, &init, 1, 1000);

}

/**
  * @brief  Reading all compensation words from calib registers
  *
  * @param  None
  *
  * @retval None
  */
void Calibdata_BME280(void){
	uint8_t CalibrationData1[26];
	uint8_t CalibrationData2[7];

	HAL_I2C_Mem_Read(&bme_i2c, BME280_ADDR, CALIB_DATA00_25_BASEADDR, 1, CalibrationData1, 26, HAL_MAX_DELAY); //From 0x88 to 0xA1
	HAL_I2C_Mem_Read(&bme_i2c, BME280_ADDR, CALIB_DATA26_41_BASEADDR, 1, CalibrationData2, 7, HAL_MAX_DELAY); //From 0xE1 to 0xE7

	dig_T1 = (CalibrationData1[1]<<8) | CalibrationData1[0];
	dig_T2 = (CalibrationData1[3]<<8) | CalibrationData1[2];
	dig_T3 = (CalibrationData1[5]<<8) | CalibrationData1[4];
	dig_P1 = (CalibrationData1[7]<<8) | CalibrationData1[6];
	dig_P2 = (CalibrationData1[9]<<8) | CalibrationData1[8];
	dig_P3 = (CalibrationData1[11]<<8) | CalibrationData1[10];
	dig_P4 = (CalibrationData1[13]<<8) | CalibrationData1[12];
	dig_P5 = (CalibrationData1[15]<<8) | CalibrationData1[14];
	dig_P6 = (CalibrationData1[17]<<8) | CalibrationData1[16];
	dig_P7 = (CalibrationData1[19]<<8) | CalibrationData1[18];
	dig_P8 = (CalibrationData1[21]<<8) | CalibrationData1[20];
	dig_P9 = (CalibrationData1[23]<<8) | CalibrationData1[22];
	dig_H1 = (CalibrationData1[25]);

	dig_H2 = (CalibrationData2[1]<<8) | CalibrationData2[0];
	dig_H3 = (CalibrationData2[2]);
	dig_H4 = (CalibrationData2[3]<<4) | (CalibrationData2[4] & 0x0F);
	dig_H5 = (CalibrationData2[5]<<4) | (CalibrationData2[4]>>4);
	dig_H6 = (CalibrationData2[6]);
}

/**
  * @brief  Reading temperature, pressure and humidity raw datas
  *
  * @param  None
  *
  * @retval None
  */
Raw_Data_t RawdataBME280(void){

	uint8_t rawData[8];
	Raw_Data_t data;

	HAL_I2C_Mem_Read(&bme_i2c, BME280_ADDR, RAWDATA_BASEADDR, 1, rawData, 8, 1000);

	//Separation of raw data buffer for temperature, humidity and pressure
	data.pressr = (rawData[0]<<12)|(rawData[1]<<4)|(rawData[2]>>4);
	data.tempr = (rawData[3]<<12)|(rawData[4]<<4)|(rawData[5]>>4);
	data.humr = (rawData[6]<<8)|(rawData[7]);

	return data;
}

/**
  * @brief  Compensation formula for temperature
  *
  * @param  Raw temperature data
  *
  * @retval Processed temperature data
  */
int32_t BME280_measure_Temp(int32_t adc_T)
{
	int32_t var1, var2, T;

	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1)))>> 12) *((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

/**
  * @brief  Compensation formula for pressure
  *
  * @param  Raw pressure data
  *
  * @retval Processed pressure data
  */
uint32_t BME280_measure_Press(int32_t adc_P)
{
	int64_t var1, var2, p;

	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
	var2 = var2 + (((int64_t)dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
	if (var1 == 0)
	{
		return 0;
	}
	p = 1048576-adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
	return (uint32_t)p;
}

/**
  * @brief  Compensation formula for humidity
  *
  * @param  Raw humidity data
  *
  * @retval Processed humidity data
  */
uint32_t BME280_measure_Hum(int32_t adc_H)
{
	int32_t v_x1_u32r;

	v_x1_u32r = (t_fine - ((int32_t)76800));
	v_x1_u32r = (((((adc_H << 14) - (((int32_t)dig_H4) << 20) - (((int32_t)dig_H5) *\
			v_x1_u32r)) + ((int32_t)16384)) >> 15) * (((((((v_x1_u32r *\
					((int32_t)dig_H6)) >> 10) * (((v_x1_u32r * ((int32_t)dig_H3)) >> 11) +\
							((int32_t)32768))) >> 10) + ((int32_t)2097152)) * ((int32_t)dig_H2) +\
					8192) >> 14));
	v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) *\
			((int32_t)dig_H1)) >> 4));
	v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
	v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
	return (uint32_t)(v_x1_u32r>>12);
}

/**
  * @brief  Reading temperature, pressure and humidity and calculating altitude from pressure value
  *
  * @param  None
  *
  * @retval BME280 sensor datas in structure data type
  */
void BME280Calculation(BME280_Data_t *result)
{
	Raw_Data_t rawData = RawdataBME280();

	Calibdata_BME280();

	result->Temperature = (BME280_measure_Temp(rawData.tempr))/100.0;	//Degress
	result->Pressure = (BME280_measure_Press(rawData.pressr))/25600.0;	//hPa
	result->Humidity = (BME280_measure_Hum(rawData.humr))/1024.0;		//%RH

	result->AltitudeP = 44330*(1-pow(result->Pressure/1013.25,1/5.255)); 	/*Calculation of altitude parameter in meters with
													simplified atmospheric pressure formula*/

	result->AltitudeTP = ((log(result->Pressure/1013.25)*(288.16)*(-8.314))/(28.97*9.81))*1000; /*Calculation of altitude parameter in
																		                    meters with atmospheric pressure formula*/
}

/**
  * @brief  Initialization of BME280
  *
  * @param  BME280Init argument to a BME280_Init_t structure that contains
  *         the configuration information for the BME280 device.
  *
  * @retval None
  */
void BME280Init(BME280_Init_t BME280Init){

	uint8_t init=0;

	//Setting it to sleep mode because the config register can only be changed while the BME280 is in sleep mode
	if(BME280_SleepMode() == HAL_OK){

		printf("BME280 was put into sleep mode so that the config register could be set.!\n");
		//Configuration of config register which is control standby time, filter and SPI 3-wire interface
		init = ((BME280Init.T_StandBy << 5) | (BME280Init.Filter << 2) | (BME280Init.SPI_EnOrDıs << 0));
		HAL_I2C_Mem_Write(&bme_i2c, BME280_ADDR, CONFIG_REG_ADDR, 1, &init, 1, 1000);
		HAL_Delay (100);
		init=0;
	}

	//Configuration of ctrl_hum register which is control oversamplig of Humidity
	init = ((BME280Init.OverSampling_H << 0) & 0x7);
	HAL_I2C_Mem_Write(&bme_i2c, BME280_ADDR, CTRL_HUM_REG_ADDR, 1, &init, 1, 1000);
	HAL_Delay (100);
	init=0;

	//Configuration of ctrl_meas register which is control oversamplig of Temperature-Pressure and device mode
	init = (BME280Init.OverSampling_T << 5) | (BME280Init.OverSampling_P << 2) | BME280Init.Mode;
	HAL_I2C_Mem_Write(&bme_i2c, BME280_ADDR, CTRL_MEAS_REG_ADDR, 1, &init, 1, 1000);
	HAL_Delay (100);
	init=0;

	printf("BME280 Initialization process is done!\n");
}

