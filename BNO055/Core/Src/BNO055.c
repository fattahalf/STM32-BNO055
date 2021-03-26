/*
 * BNO055.c
 *
 *  Created on: Mar 25, 2021
 *      Author: Fattah .Alf
 */
#include <BNO055.h>

extern I2C_HandleTypeDef hi2c1;
extern DMA_HandleTypeDef hdma_i2c1_rx;
extern DMA_HandleTypeDef hdma_i2c1_tx;

uint8_t GPwrMode = NormalG;   		// Gyroscope power mode
uint8_t Gscale = GFS_2000DPS; 		// Gyroscope full scale

//uint8_t Godr	= GODR_250Hz; 	 	// Gyroscope sample rate
uint8_t Gbw = GBW_230Hz;    		// Gyroscope bandwidth
uint8_t Ascale = AFS_16G;      		// accelerometer full scale
uint8_t APwrMode = NormalA;   		// Accelerometer power mode
uint8_t Abw = ABW_250Hz;    		// Accelerometer bandwidth, accelerometer sample rate divided by ABW_divx
//
//uint8_t Mscale 	= MFS_4Gauss;		// Select magnetometer full-scale resolution
uint8_t MOpMode = EnhancedRegular;    	// Select magnetometer perfomance mode
uint8_t MPwrMode = Normal;    			// Select magnetometer power mode
uint8_t Modr = MODR_30Hz;    			// Select magnetometer ODR when in BNO055 bypass mode

uint8_t PWRMode = Normalpwr;  			// Select BNO055 power mode
uint8_t OPRMode = IMU;    				// specify operation mode for sensors [ACCONLY|MAGONLY|GYROONLY|ACCMAG|ACCGYRO|MAGGYRO|AMG|NDOF|NDOF_FMC_OFF]

uint8_t status;               			// BNO055 data status register
float aRes, gRes, mRes; 				// scale resolutions

// IMU calibration variables
uint8_t cal_sys 	= 0;
uint8_t cal_gyro 	= 0;
uint8_t cal_acc 	= 0;
uint8_t cal_mag 	= 0;
uint8_t cal_imu 	= 0;

const uint8_t num_of_bytes_read = 18;		// Read number of bytes from IMU (24 for ACCGYRO; 38 for NDOF)

const char read_devid[] 	= {START_BYTE, REG_READ, BNO055_CHIP_ID, 0x01};
//const char read_acc[] 		= {REG_READ, BNO055_ACC_DATA_X_LSB, num_of_bytes_read};
const char read_calib[2] 	= {REG_READ, BNO055_CALIB_STAT};
const char reset_sensor[3]	= {REG_WRITE, BNO055_SYS_TRIGGER, 0x01 << 5};
uint8_t get_readings[1] 	= {BNO055_EUL_HEADING_LSB};
uint8_t get_readingX 	= {BNO055_QUA_DATA_X_LSB};
uint8_t get_readingY 	= {BNO055_QUA_DATA_Y_LSB};
uint8_t get_readingZ 	= {BNO055_QUA_DATA_Z_LSB};
uint8_t get_readingW 	= {BNO055_QUA_DATA_W_LSB};

// Configure BNO sensor
void BNO055_Init_I2C(I2C_HandleTypeDef* hi2c_device) {
	// Select BNO055 config mode
	uint8_t opr_config_mode[2] = {BNO055_OPR_MODE, CONFIGMODE};
	HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, opr_config_mode, sizeof(opr_config_mode), 10);
	HAL_Delay(10);

	// Select page 1 to configure sensors
	uint8_t conf_page1[2] = {BNO055_PAGE_ID, 0x01};
	HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, conf_page1, sizeof(conf_page1), 10);
	HAL_Delay(10);

	// Configure ACC (Page 1; 0x08)
	uint8_t conf_acc[2] = {BNO055_ACC_CONFIG, APwrMode << 5 | Abw << 2 | Ascale};
	HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, conf_acc, sizeof(conf_acc), 10);
	HAL_Delay(10);

	// Configure GYR
	uint8_t conf_gyro[2] = {BNO055_GYRO_CONFIG_0, Gbw << 3 | Gscale};
	HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, conf_gyro, sizeof(conf_gyro), 10);
	HAL_Delay(10);

	uint8_t conf_gyro_pwr[2] = {BNO055_GYRO_CONFIG_1, GPwrMode};
	HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, conf_gyro_pwr, sizeof(conf_gyro_pwr), 10);
	HAL_Delay(10);

	// Configure MAG
	uint8_t conf_mag_pwr[4] = {REG_WRITE, BNO055_MAG_CONFIG, 0x01, MPwrMode << 5 | MOpMode << 3 | Modr};
	HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, conf_mag_pwr, sizeof(conf_mag_pwr), 10);
	HAL_Delay(10);

	// Configure EULER
//	uint8_t conf_mag_pwr[4] = {REG_WRITE, BNO055_MAG_CONFIG, 0x01, MPwrMode << 5 | MOpMode << 3 | Modr};
//	HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, conf_mag_pwr, sizeof(conf_mag_pwr), 10);
//	HAL_Delay(10);
//	// Select BNO055 gyro temperature source
	//PutHexString(START_BYTE, BNO055_TEMP_SOURCE, 0x01 );

	// Select page 0
	uint8_t conf_page0[2] = {BNO055_PAGE_ID, 0x00};
	HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, conf_page0, sizeof(conf_page0), 10);
	HAL_Delay(10);

	// Select BNO055 sensor units (Page 0; 0x3B, default value 0x80)
	/*- ORIENTATION_MODE		 - Android					(default)
		- VECTOR_ACCELEROMETER - m/s^2  					(default)
		- VECTOR_MAGNETOMETER  - uT								(default)
		- VECTOR_GYROSCOPE     - rad/s        v		(must be configured)
		- VECTOR_EULER         - degrees					(default)
		- VECTOR_LINEARACCEL   - m/s^2        v		(default)
		- VECTOR_GRAVITY       - m/s^2						(default)
	*/
	//const char conf_units[4] = {REG_WRITE, BNO055_UNIT_SEL, 0x01, 0x82};
	//SendAccelData(USART1, (uint8_t*)conf_units);
	//HAL_Delay(50);

	// Select BNO055 system power mode (Page 0; 0x3E)
	uint8_t pwr_pwrmode[2] = {BNO055_PWR_MODE, PWRMode};
	HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, pwr_pwrmode, sizeof(pwr_pwrmode), 10);
	HAL_Delay(10);

	// Select BNO055 system operation mode (Page 0; 0x3D)
	uint8_t opr_oprmode[2] = {BNO055_OPR_MODE, OPRMode};
	HAL_I2C_Master_Transmit(hi2c_device, BNO055_I2C_ADDR_LO<<1, opr_oprmode, sizeof(opr_oprmode), 10);
	HAL_Delay(50);
}



// Send data to BNO055 over I2C
//uint8_t GetAccelData(I2C_HandleTypeDef* hi2c_device, uint8_t* str) {
//	uint8_t status;
//	status = HAL_I2C_Mem_Read(hi2c_device, BNO055_I2C_ADDR_LO<<1, BNO055_ACC_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, str, IMU_NUMBER_OF_BYTES,100);
//  //while (HAL_I2C_GetState(hi2c_device) != HAL_I2C_STATE_READY) {}
//	return status;
uint8_t GetEulData(I2C_HandleTypeDef* hi2c_device, uint8_t* str) {
	uint8_t status;
	status = HAL_I2C_Mem_Read(hi2c_device, BNO055_I2C_ADDR_LO<<1, BNO055_EUL_HEADING_LSB, I2C_MEMADD_SIZE_8BIT, str, IMU_NUMBER_OF_BYTES,100);
	//while (HAL_I2C_GetState(hi2c_device) != HAL_I2C_STATE_READY) {}
	return status;
}
uint8_t GetQuaDataX(I2C_HandleTypeDef* hi2c_device, uint8_t* str) {
	uint8_t status;
	status = HAL_I2C_Mem_Read(hi2c_device, BNO055_I2C_ADDR_LO<<1, BNO055_QUA_DATA_X_LSB, I2C_MEMADD_SIZE_8BIT, str, IMU_NUMBER_OF_BYTES,100);
	//while (HAL_I2C_GetState(hi2c_device) != HAL_I2C_STATE_READY) {}
	return status;
}
uint8_t GetQuaDataY(I2C_HandleTypeDef* hi2c_device, uint8_t* str) {
	uint8_t status;
	status = HAL_I2C_Mem_Read(hi2c_device, BNO055_I2C_ADDR_LO<<1, BNO055_QUA_DATA_Y_LSB, I2C_MEMADD_SIZE_8BIT, str, IMU_NUMBER_OF_BYTES,100);
	//while (HAL_I2C_GetState(hi2c_device) != HAL_I2C_STATE_READY) {}
	return status;
}
uint8_t GetQuaDataZ(I2C_HandleTypeDef* hi2c_device, uint8_t* str) {
	uint8_t status;
	status = HAL_I2C_Mem_Read(hi2c_device, BNO055_I2C_ADDR_LO<<1, BNO055_QUA_DATA_Z_LSB , I2C_MEMADD_SIZE_8BIT, str, IMU_NUMBER_OF_BYTES,100);
	//while (HAL_I2C_GetState(hi2c_device) != HAL_I2C_STATE_READY) {}
	return status;
}
uint8_t GetQuaDataW(I2C_HandleTypeDef* hi2c_device, uint8_t* str) {
	uint8_t status;
	status = HAL_I2C_Mem_Read(hi2c_device, BNO055_I2C_ADDR_LO<<1, BNO055_QUA_DATA_W_LSB , I2C_MEMADD_SIZE_8BIT, str, IMU_NUMBER_OF_BYTES,100);
	//while (HAL_I2C_GetState(hi2c_device) != HAL_I2C_STATE_READY) {}
	return status;
}
// TBD
/*void readAccelData(int16_t *destination) {
  uint8_t rawData[6];  // x/y/z accel register data stored here
  destination[0] = ((int16_t)rawData[1] << 8) | rawData[0] ;      // Turn the MSB and LSB into a signed 16-bit value
  destination[1] = ((int16_t)rawData[3] << 8) | rawData[2] ;
  destination[2] = ((int16_t)rawData[5] << 8) | rawData[4] ;
}*/


// TBD
uint8_t GetEulChipId(I2C_HandleTypeDef* hi2c_device, uint8_t *chip_id) {
	return HAL_I2C_Mem_Read(hi2c_device, BNO055_I2C_ADDR_LO<<1, BNO055_CHIP_ID, I2C_MEMADD_SIZE_8BIT, chip_id, 1, 100);
}


// TBD
uint8_t GetEulTemp(I2C_HandleTypeDef* hi2c_device) {
	uint8_t temp;
	HAL_I2C_Mem_Read_DMA(hi2c_device, BNO055_I2C_ADDR_LO<<1, BNO055_TEMP, I2C_MEMADD_SIZE_8BIT, &temp, 1);
	while (HAL_I2C_GetState(hi2c_device) != HAL_I2C_STATE_READY) {}
	return temp;
}


// Get IMU calibration values
uint8_t BNO055_Get_Calibration(I2C_HandleTypeDef* hi2c_device) {
	uint8_t calibration;
	HAL_I2C_Mem_Read_DMA(hi2c_device, BNO055_I2C_ADDR_LO<<1, BNO055_CALIB_STAT, I2C_MEMADD_SIZE_8BIT, &calibration, 1);
	while (HAL_I2C_GetState(hi2c_device) != HAL_I2C_STATE_READY);
	return calibration;
}


// Calculate IMU calibration values
void BNO055_Calc_Calibration(uint8_t calibration, uint8_t *cal_system, uint8_t *cal_gyro, uint8_t *cal_acc, uint8_t *cal_mag) {
	*cal_system = (calibration >> 6) & 0x03;
	*cal_gyro 	= (calibration >> 4) & 0x03;
	*cal_acc 		= (calibration >> 2) & 0x03;
	*cal_mag 		= (calibration) & 0x03;
}

void getIMU(I2C_HandleTypeDef* hi2c_device, float __eul_data[3], float __qua_data[3]) {
	  GetEulData(&hi2c1, (uint8_t*)imu_readings);
	  uint16_t __euler_h__ = (((int16_t)((uint8_t *)(imu_readings))[1] << 8) | ((uint8_t *)(imu_readings))[0]);      // Turn the MSB and LSB into a signed 16-bit value
	  uint16_t __euler_p__ = (((int16_t)((uint8_t *)(imu_readings))[3] << 8) | ((uint8_t *)(imu_readings))[2]);
	  uint16_t __euler_r__ = (((int16_t)((uint8_t *)(imu_readings))[5] << 8) | ((uint8_t *)(imu_readings))[4]);

	  __eul_data[0] = ((float)(__euler_h__)) / 16.0f;
	  __eul_data[1] = ((float)(__euler_p__)) / 16.0f;
	  __eul_data[2] = ((float)(__euler_r__)) / 16.0f;

	  GetQuaDataX(&hi2c1, (uint8_t*)imu_readings);
	  uint16_t __qua_x__ = (((int16_t)((uint8_t *)(imu_readings))[1] << 8) | ((uint8_t *)(imu_readings))[0]);
	  __qua_data[0] =  ((float)(__qua_x__))/16384;

	  GetQuaDataY(&hi2c1, (uint8_t*)imu_readings);
	  uint16_t __qua_y__ = (((int16_t)((uint8_t *)(imu_readings))[3] << 8) | ((uint8_t *)(imu_readings))[2]);
	  __qua_data[1] = ((float)(__qua_y__))/16384;

	  GetQuaDataZ(&hi2c1, (uint8_t*)imu_readings);
	  uint16_t __qua_z__ = (((int16_t)((uint8_t *)(imu_readings))[5] << 8) | ((uint8_t *)(imu_readings))[4]);
	  __qua_data[2] = ((float)(__qua_z__))/16384;

	  GetQuaDataW(&hi2c1, (uint8_t*)imu_readings);
	  uint16_t __qua_w__ = (((int16_t)((uint8_t *)(imu_readings))[7] << 8) | ((uint8_t *)(imu_readings))[6]);
	  __qua_data[3] =  ((float)(__qua_w__))/16384;
}

