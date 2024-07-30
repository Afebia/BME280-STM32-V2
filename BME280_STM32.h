extern I2C_HandleTypeDef hi2c1;
#define bme_i2c (hi2c1)

#define BME280_ADDR 				(0x76 << 1) 		/*!I2C Address of BME280, which is 0xEC*/
#define CHIP_ID_BME 				0x60				/*!Chip ID of BME280*/

/*
 *	 ===============================================================================
 *                     	   ##### BME280 Register address Macros #####
 *	 ===============================================================================
 */
#define HUM_LSB_REG_ADDR 			0xFE  				/*!Contains LSB part of raw humidity data */
#define HUM_MSB_REG_ADR 			0xFD				/*!Contains MSB part of raw humidity data */
#define TEMP_XLSB_REG_ADDR 			0xFC				/*!Contains XLSB part of raw temperature data, depends on pressure resuliton */
#define TEMP_LSB_REG_ADDR 			0xFB				/*!Contains LSB part of raw temperature data */
#define TEMP_MSB_REG_ADDR 			0xFA				/*!Contains MSB part of raw temperature data */
#define PRESS_XLSB_REG_ADDR 		0xF9				/*!Contains XLSB part of raw pressure data, depends on temperature resuliton */
#define PRESS_LSB_REG_ADDR 			0xF8				/*!Contains LSB part of raw pressure data */
#define PRESS_MSB_REG_ADDR 			0xF7				/*!Contains MSB part of raw pressure data */
#define RAWDATA_BASEADDR 			PRESS_MSB_REG_ADDR	/*!Base address of Raw data registers */

#define CONFIG_REG_ADDR 			0xF5    			/*!Sets the rate, filter and enable or disable 3-wire SPI interface*/
#define CTRL_MEAS_REG_ADDR 			0xF4 				/*!Control oversampling of temp and press and changes sensor modes */
#define STATUS_REG_ADDR				0xF3				/*!Contains 2 bits which indicate status of BME280*/
#define RESET_REG_ADDR 				0xE0    			/*!BME280 RESET register, to reset BME280, 0xB6 must be written to this register*/
#define CHIP_ID_REG_ADDR 			0xD0 				/*!Contains chip number of BME280, which is 0x60*/

#define CTRL_HUM_REG_ADDR 			0xF2  				/*!Controls oversampling of humidity data.
														 * NOTE: This register must be written before changing ctrl_meas register
														 */

#define CALIB_DATA00_25_BASEADDR 	0x88  				/*!Base address of calib registers which contains compensation words from 0 to 25*/
#define CALIB_DATA26_41_BASEADDR 	0xE1  				/*!Base address of calib registers which contains compensation words from 26 to 41*/


/*
 *	 ===============================================================================
 *                     	   ##### BME280 Register definition #####
 *	 ===============================================================================
 *
 *
 * All possible OVERSAMPLING amount for pressure, temperatur and humidity
 */
#define OVERSAMPLING_SKIPPED  		0x0
#define OVERSAMPLING_1				0x1
#define OVERSAMPLING_2				0x2
#define OVERSAMPLING_4				0x3
#define OVERSAMPLING_8				0x4
#define OVERSAMPLING_16				0x5

/*
 * Sensor modes	macros
 */
#define BME280_SLEEP_MODE  			0x0
#define BME280_FORCED_MODE 			0x1
#define BME280_NORMAL_MODE 			0x3

/*
 * Standby time macros
 */
#define T_SB_0_5  					0
#define T_SB_62_5					1
#define T_SB_125					2
#define T_SB_250					3
#define T_SB_500					4
#define T_SB_1000					5
#define T_SB_10						6
#define T_SB_20						7

/*
 *Fiter	macros
 */
#define FILTER_OFF  				0x0
#define FILTER_2					0x1
#define FILTER_4					0x2
#define FILTER_8					0x3
#define FILTER_16					0x4

/*
 * Enable or disable 3-wire SPI interface macros
 */
#define SPI3_W_ENABLE				0x1
#define SPI3_W_DISABLE				0x0

/**
  * BME280 raw data structure definition
  */
typedef struct{
	int32_t tempr;
	int32_t pressr;
	int32_t humr;
}Raw_Data_t;

typedef struct {
    float Temperature;
    float Pressure;
    float Humidity;
    float AltitudeP;
    float AltitudeTP;
}BME280_Data_t;

/**
  * BME280 Init structure definition
  */
typedef struct
{
	uint8_t OverSampling_T;
	uint8_t OverSampling_P;
	uint8_t OverSampling_H;
	uint8_t Mode;
	uint8_t T_StandBy;
	uint8_t Filter;
	uint8_t SPI_EnOrDıs;
}BME280_Init_t;

/*
 * BME280 library function declaration
 */
void Calibdata_BME280(void);
Raw_Data_t RawdataBME280(void);
void BME280Init(BME280_Init_t BME280Init);
void BME280Calculation(BME280_Data_t *result);
HAL_StatusTypeDef BME280_SleepMode(void);
uint32_t BME280_measure_Hum(int32_t adc_H);
uint32_t BME280_measure_Press(int32_t adc_P);
int32_t BME280_measure_Temp(int32_t adc_T);

void Reset_BME280(void);

#endif /* INC_BME280_STM32_H_ */
