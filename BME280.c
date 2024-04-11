/*
 * BME280.c
 *
 *  Created on: Mar 6, 2024
 *      Author: Berat Bayram
 */
#include "BME280_STM32.h"

unsigned short dig_T1,dig_P1, dig_H1, dig_H3;
signed short dig_T2, dig_T3,dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9, dig_H2,dig_H4, dig_H5, dig_H6;

raw_data_t data;

float Temperature, Pressure, Humidity,AltitudeP,AltitudeTP;

void Reset_BME280(void){

	uint8_t data = 0xB6;
	HAL_I2C_Mem_Write(&hi2c1, BME280_ADDR, RESET_P, 1, &data, 1, HAL_MAX_DELAY);
	HAL_Delay(200);

	uint8_t id;
	HAL_I2C_Mem_Read(&hi2c1, BME280_ADDR, BME280_ID_P, 1, &id, 1, HAL_MAX_DELAY);
	if(id != CHIP_ID_BME){
		HAL_Delay(1000);
	}
}

//Compensation değerlerinin hesaplanması
void Calibdata_BME280(void){
	uint8_t CalibrationData1[25];
	uint8_t CalibrationData2[7];
	HAL_I2C_Mem_Read(&hi2c1, BME280_ADDR, CALIB_DATA1_P, 1, CalibrationData1, 25, HAL_MAX_DELAY); //0x88 ile 0xA1
	HAL_I2C_Mem_Read(&hi2c1, BME280_ADDR, CALIB_DATA2_P, 1, CalibrationData2, 7, HAL_MAX_DELAY); //0xE1 ile 0xE7
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
	dig_H1 = (CalibrationData1[24]);

	dig_H2 = (CalibrationData2[1]<<8) | CalibrationData2[0];
	dig_H3 = (CalibrationData2[2]);
	dig_H4 = (CalibrationData2[3]<<4) | (CalibrationData2[4] & 0x0f);
	dig_H5 = (CalibrationData2[5]<<4) | (CalibrationData2[4]>>4);
	dig_H6 = (CalibrationData2[6]);

}

void RawdataBME280(void){
	uint8_t rawData[8];

	// 0xF7 ile 0xFE arasındaki raw dataların okunması
	HAL_I2C_Mem_Read(&hi2c1, BME280_ADDR, PRESS_MSB_P, 1, rawData, 8, 1000);

	// Sıcaklık, nem ve basınç için raw datanın ayrılması
	//press ve temp 8+8+4=20 bit, hum 8+8=16 bit
	data.pressr = (rawData[0]<<12)|(rawData[1]<<4)|(rawData[2]>>4);
	data.tempr = (rawData[3]<<12)|(rawData[4]<<4)|(rawData[5]>>4);
	data.humr = (rawData[6]<<8)|(rawData[7]);

}

int32_t t_fine;

//Sıcaklık Ölçümü
int32_t BME280_measure_Temp(int32_t adc_T)
{

	Calibdata_BME280();
	int32_t var1, var2, T;
	var1 = ((((adc_T>>3) - ((int32_t)dig_T1<<1))) * ((int32_t)dig_T2)) >> 11;
	var2 = (((((adc_T>>4) - ((int32_t)dig_T1)) * ((adc_T>>4) - ((int32_t)dig_T1)))>> 12) *((int32_t)dig_T3)) >> 14;
	t_fine = var1 + var2;
	T = (t_fine * 5 + 128) >> 8;
	return T;
}

//Basınç Ölçümü
uint32_t BME280_measure_Press(int32_t adc_P)
{

	Calibdata_BME280();
	int64_t var1, var2, p;
	var1 = ((int64_t)t_fine) - 128000;
	var2 = var1 * var1 * (int64_t)dig_P6;
	var2 = var2 + ((var1*(int64_t)dig_P5)<<17);
	var2 = var2 + (((int64_t)dig_P4)<<35);
	var1 = ((var1 * var1 * (int64_t)dig_P3)>>8) + ((var1 * (int64_t)dig_P2)<<12);
	var1 = (((((int64_t)1)<<47)+var1))*((int64_t)dig_P1)>>33;
	if (var1 == 0)
	{
		return 0; // avoid exception caused by division by zero
	}
	p = 1048576-adc_P;
	p = (((p<<31)-var2)*3125)/var1;
	var1 = (((int64_t)dig_P9) * (p>>13) * (p>>13)) >> 25;
	var2 = (((int64_t)dig_P8) * p) >> 19;
	p = ((p + var1 + var2) >> 8) + (((int64_t)dig_P7)<<4);
	return (uint32_t)p;
}

//Nem Ölçümü
uint32_t BME280_measure_Hum(int32_t adc_H)
{

	Calibdata_BME280();
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

//Sıcaklık,nem ve basınç ölçme fonksiyonu
void BME280Calculation(void){

	RawdataBME280();

	//Sıcaklık,nem ve basınç ölçme
		Temperature = (BME280_measure_Temp(data.tempr))/100.0;//Derece

		Pressure = (BME280_measure_Press(data.pressr))/25600.0;//hPa

		Humidity = (BME280_measure_Hum(data.humr))/1024.0;//%RH

		AltitudeP = 44330*(1-pow(Pressure/1013.25,1/5.255)); //Metre cinsinden yükseklik sadece basınç verisi ile yükseklik

		AltitudeTP = ((log(Pressure/1013.25)*(288.16)*(-8.314))/(28.97*9.81))*1000; // Metre cinsinden
		                                   	   	   	   	   	   	   	   	   	   	   	// sıcaklık ve basınç verisi ile yükseklik
}

//İnitilization
void BME280Init(oversampling_t oversamplingT , oversampling_t  oversamplingP,
		oversampling_t oversamplingH,sensor_mode_t mode,standby_t t_sb,filter_t filter){

	uint8_t init;

	init = SLEEP; //Config registerı Sleep modunda değiştirilebilir.
	HAL_I2C_Mem_Write(&hi2c1, BME280_ADDR, CTRL_MEAS_P, 1, &init, 1, 1000);
	HAL_Delay (100);

	//Config register, standby time, IIR filter, ve SPI 3-wire enable biti(1 ise aktif)
	init = (t_sb << 5)| (filter << 2) | 0x00;
	HAL_I2C_Mem_Write(&hi2c1, BME280_ADDR, CONFIG_P, 1, &init, 1, 1000);
	HAL_Delay (100);

	//CTRL_hum register, Nem oversampling
	init = oversamplingH;
	HAL_I2C_Mem_Write(&hi2c1, BME280_ADDR, CTRL_HUM_P, 1, &init, 1, 1000);
	HAL_Delay (100);


	//CTRL_meas register, Sıcaklık ve basınç oversampling ve mode select
	init = (oversamplingT << 5) | (oversamplingP << 2) | mode;
	HAL_I2C_Mem_Write(&hi2c1, BME280_ADDR, CTRL_MEAS_P, 1, &init, 1, 1000);
	HAL_Delay (100);

}

