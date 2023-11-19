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
  bool isConnected;
  Acceleration acceleration;
  AngularVelocity angularVelocity;
} MPU6050;


MPU6050 mpu6050;

static const uint8_t i2c_ch = _DEF_I2C1;


#ifdef _USE_HW_CLI
static void cliMPU6050(cli_args_t *args);
#endif



bool mpu6050_init(void){
	bool ret = false;

	//uint8_t mpu6050_isReady =0;
	//bool isConnected = false;
	//isConnected = i2cReadByte(i2c_ch, MPU6050_ADDR, WHO_AM_I_REG, &mpu6050_isReady, 1000);
	//if (mpu6050_isReady == 0x68){
	//	ret = true;
	//	mpu6050.isConnected = true;
	//} TODO: WHO_AM_I_REG 작동이 이상함 나중에 점검 필요
	mpu6050_configuration();
#ifdef _USE_HW_CLI
cliAdd("mpu6050", cliMPU6050);
#endif

	return ret;
}

bool mpu6050_configuration(){
		bool ret = false;

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

	mpu6050.acceleration.Ax = Accel_X_RAW/16384.0;
	mpu6050.acceleration.Ay = Accel_Y_RAW/16384.0;
	mpu6050.acceleration.Az = Accel_Z_RAW/16384.0;
}


void mpu6050_read_gyro(void)
{
	uint8_t rec_data[6];

	// Read 6 BYTES of data starting from GYRO_XOUT_H register
	i2cReadBytes(i2c_ch, MPU6050_ADDR, GYRO_XOUT_H_REG, rec_data, 6, 1000);

	int16_t Gyro_X_RAW = (int16_t)(rec_data[0] << 8 | rec_data [1]);
	int16_t Gyro_Y_RAW = (int16_t)(rec_data[2] << 8 | rec_data [3]);
	int16_t Gyro_Z_RAW = (int16_t)(rec_data[4] << 8 | rec_data [5]);

	/*** convert the RAW values into dps (degree/s)
	     we have to divide according to the Full scale value set in FS_SEL
	     I have configured FS_SEL = 0. So I am dividing by 131.0
	     for more details check GYRO_CONFIG Register              ****/

	mpu6050.angularVelocity.Gx = Gyro_X_RAW/131.0;
	mpu6050.angularVelocity.Gy = Gyro_Y_RAW/131.0;
	mpu6050.angularVelocity.Gz = Gyro_Z_RAW/131.0;
}

#ifdef _USE_HW_CLI
void cliMPU6050(cli_args_t *args){
	  bool ret = true;
	  bool i2c_ret;
	  uint8_t i;
	  uint32_t pre_time;


	  if (args->argc == 1)
	  {
	    if(args->isStr(0, "scan") == true)
	    {
	      for (i=0x00; i<= 0x7F; i++)
	      {
	        if (i2cIsDeviceReady(i2c_ch, i) == true)
	        {
	          cliPrintf("I2C Addr 0x%X : OK\n", i);
	        }
	      }
	    }
	    else if(args->isStr(0, "open") == true){
	      i2c_ret = i2cOpen(0, 400);
	      if (i2c_ret == true)
	      {
	        cliPrintf("I2C Open OK\n");
	      }
	      else
	      {
	        cliPrintf("I2C Open Fail\n");
	      }
	    }
	    else if (args->isStr(0, "config") == true){
	    	bool mpu6050_ret = false;

	    	mpu6050_ret= mpu6050_configuration();
	    	if (mpu6050_ret == true){
	    		cliPrintf("MPU6050 configuration was done\n");
	    	}
	    	else {
	    		cliPrintf("MPU6050 configuration was failed\n");
	    	}
	    }
	    if(args->isStr(0, "get_accel") == true)
	        {
	    	while(cliKeepLoop()){

	    	      if (millis()-pre_time >= 100)
	    	      {
	    	        pre_time = millis();
		    		mpu6050_read_accel();
		    		float Ax, Ay, Az;
		    		Ax = mpu6050.acceleration.Ax;
		    		Ay = mpu6050.acceleration.Ay;
		    		Az = mpu6050.acceleration.Az;
		    		cliPrintf("[Acceleration] x: %f, y: %f, z: %f\n", Ax, Ay, Az);
	    	      }

	    	}
	        }
	    if(args->isStr(0, "get_angular") == true)
	        {
	    	while(cliKeepLoop()){
	    	      if (millis()-pre_time >= 100)
	    	      {
	    	        pre_time = millis();
	    		mpu6050_read_gyro();
	    		float Gx, Gy, Gz;
	    		Gx = mpu6050.angularVelocity.Gx;
	    		Gy = mpu6050.angularVelocity.Gy;
	    		Gz = mpu6050.angularVelocity.Gz;
	    		cliPrintf("[Angular Velocity] x: %f, y: %f, z: %f\n", Gx, Gy, Gz);
	    		}
	    	}
	    	}

	  }
	  else
	  {
	    ret = false;
	  }


	  if (ret == false)
	  {
	    cliPrintf( "mcp4725 open \n");
	    cliPrintf( "mcp4725 scan \n");
	    cliPrintf( "mcp4725 config \n");
	    cliPrintf( "mcp4725 get_accel \n");
	    cliPrintf( "mcp4725 get_angular \n");

	  }

	}

#endif


