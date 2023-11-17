/*
 * mpu6050.c
 *
 *  Created on: 2023. 11. 17.
 *      Author: User
 */


#include "mpu6050.h"
#include "cli.h"

typedef struct MPU6050
{
  I2C_HandleTypeDef *hi2c;
  bool isConnected;
  float Ax, Ay, Az, Gx, Gy, Gz;
} MPU6050;


extern I2C_HandleTypeDef hi2c1;
MPU6050 mpu6050;

const uint8_t i2c_ch = _DEF_I2C1;


#ifdef _USE_HW_CLI
static void cliMPU6050(cli_args_t *args);
#endif



bool mpu6050_init(void){
	bool ret = false;

	mpu6050.hi2c = &hi2c1;
	uint8_t mpu6050_isReady;
	i2cReadByte(i2c_ch, MPU6050_ADDR, WHO_AM_I_REG, &mpu6050_isReady, 1000);
	if (mpu6050_isReady == 104){
		ret = true;
		mpu6050.isConnected = true;
	}

#ifdef _USE_HW_CLI
cliAdd("mpu6050", cliMPU6050);
#endif

	return ret;
}

bool mpu6050_configuration(){
	bool ret;
	if (mpu6050.isConnected == true){
		// power management register 0X6B we should write all 0's to wake the sensor up
		uint8_t write_bits = 0x00;
		ret = i2cWriteByte(i2c_ch, MPU6050_ADDR, PWR_MGMT_1_REG, write_bits, 1000);

		// Set DATA RATE of 1KHz by writing SMPLRT_DIV register
		write_bits = 0x07;
		ret = i2cWriteByte(i2c_ch, MPU6050_ADDR, SMPLRT_DIV_REG, write_bits, 1000);

		// Set accelerometer configuration in ACCEL_CONFIG Register
		// XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> +- 2g
		write_bits = 0x00;
		ret= i2cWriteByte(i2c_ch, MPU6050_ADDR, ACCEL_CONFIG_REG, write_bits, 1000);

		// Set Gyroscopic configuration in GYRO_CONFIG Register
		// XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> +- 250 degree/s
		write_bits = 0x00;
		ret = i2cWriteByte(i2c_ch, MPU6050_ADDR, GYRO_CONFIG_REG, write_bits, 1000);
	}
	else{
		ret= false;
	}
	return ret;

}

void mpu6050_read_accel(void)
{
	uint8_t rec_data[6];

	// Read 6 BYTES of data starting from ACCEL_XOUT_H register
	i2cReadBytes(i2c_ch, MPU6050_ADDR, ACCEL_XOUT_H_REG, rec_data, 6, 1000);

	int16_t Accel_X_RAW = (int16_t)(rec_data[0] << 8 | rec_data [1]);
	int16_t Accel_Y_RAW = (int16_t)(rec_data[2] << 8 | rec_data [3]);
	int16_t Accel_Z_RAW = (int16_t)(rec_data[4] << 8 | rec_data [5]);

	/*** convert the RAW values into acceleration in 'g'
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 16384.0
	     for more details check ACCEL_CONFIG Register              ****/

	mpu6050.Ax = Accel_X_RAW/16384.0;
	mpu6050.Ay = Accel_Y_RAW/16384.0;
	mpu6050.Az = Accel_Z_RAW/16384.0;
}


void MPU6050_Read_Gyro (void)
{
	uint8_t rec_data[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H register
	i2cReadBytes(i2c_ch, MPU6050_ADDR, GYRO_XOUT_H_REG, rec_data, 6, 1000);

	int16_t Gyro_X_RAW = (int16_t)(rec_data[0] << 8 | rec_data [1]);
	int16_t Gyro_Y_RAW = (int16_t)(rec_data[2] << 8 | rec_data [3]);
	int16_t Gyro_Z_RAW = (int16_t)(rec_data[4] << 8 | rec_data [5]);

	/*** convert the RAW values into dps (�/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 131.0
	     for more details check GYRO_CONFIG Register              ****/

	mpu6050.Gx = Gyro_X_RAW/131.0;
	mpu6050.Gy = Gyro_Y_RAW/131.0;
	mpu6050.Gz = Gyro_Z_RAW/131.0;
}

#ifdef _USE_HW_CLI
void cliMPU6050(cli_args_t *args){

}

#endif


