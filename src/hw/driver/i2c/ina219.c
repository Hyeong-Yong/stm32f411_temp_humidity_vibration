/*
 * ina219.c
 *
 *  Created on: 2023. 6. 28.
 *      Author: hwang
 */


#include "ina219.h"
#include "i2c.h"
#include "cli.h"


#ifdef _USE_HW_INA219

#ifdef _USE_HW_CLI
static void cli_ina219(cli_args_t *args);
#endif


static const uint8_t i2c_ch = _DEF_I2C2; //channel 0: _DEF_I2C2;

extern I2C_HandleTypeDef hi2c3;

typedef struct INA219
{
	I2C_HandleTypeDef *hi2c;
	INA219_ADDRESS	  i2cAddress;
	uint16_t          calibrationValue;
	int16_t           currentDivider_mA;
	int16_t           powerMultiplier_mW;
	bool			  isConnected;
} INA219_t;

INA219_t  ina219;

void INA219_Init(){
	ina219.hi2c = &hi2c3;

	// the I2C address the device can be found on. //Default is 0x40
	ina219.i2cAddress = I2C_ADDRESS1;


	ina219.currentDivider_mA = 0;
	ina219.powerMultiplier_mW = 0;
	ina219.isConnected = false;

	//uint8_t ina219_isReady = HAL_I2C_IsDeviceReady(i2c, (Address << 1), 3, 2);
	uint8_t ina219_isReady = i2cIsDeviceReady(i2c_ch, ina219.i2cAddress);
	if(ina219_isReady == HAL_OK)
	{
		INA219_Reset(ina219);
		INA219_setCalibration_32V_2A(ina219);
		ina219.isConnected = true;
	}



	#ifdef _USE_HW_CLI
		cliAdd("ina219", cli_ina219);
	#endif
}

uint8_t INA219_isConnected(){
	return ina219.isConnected;
}

uint16_t INA219_Read16(uint8_t reg_addr)
{
	uint8_t ret_buf[2];
	i2cReadBytes(i2c_ch, ina219.i2cAddress, reg_addr, ret_buf, 2, 100);

	uint8_t ret = ((ret_buf[0] << 8) | ret_buf[1]);

	return ret;
}


void INA219_Write16(uint8_t reg_addr, uint16_t Value)
{
	uint8_t addr[2];
	addr[0] = (Value >> 8) & 0xff;  // upper byte
	addr[1] = (Value >> 0) & 0xff; // lower byte
	i2cWriteBytes(i2c_ch, ina219.i2cAddress, reg_addr, (uint8_t*)addr, 2, 100);
}

uint16_t INA219_ReadBusVoltage(INA219_t *ina219)
{
	uint16_t result = INA219_Read16(INA219_REG_BUSVOLTAGE);

	return ((result >> 3  ) * 4);

}

int16_t INA219_ReadCurrent_raw()
{
	int16_t result = INA219_Read16(INA219_REG_CURRENT);

	return (result );
}

int16_t INA219_ReadCurrent()
{
	int16_t result = INA219_ReadCurrent_raw();

	return (result / ina219.currentDivider_mA );
}

uint16_t INA219_ReadShuntVolage()
{
	uint16_t result = INA219_Read16(INA219_REG_SHUNTVOLTAGE);

	return (result * 0.01 );
}

void INA219_Reset()
{
	INA219_Write16(INA219_REG_CONFIG, INA219_CONFIG_RESET);
	HAL_Delay(1);
}

void INA219_setCalibration(uint16_t CalibrationData)
{
	INA219_Write16(INA219_REG_CALIBRATION, CalibrationData);
}

uint16_t INA219_getConfig()
{
	uint16_t result = INA219_Read16(INA219_REG_CONFIG);
	return result;
}

void INA219_setConfig(uint16_t Config)
{
	INA219_Write16(INA219_REG_CONFIG, Config);
}

void INA219_setCalibration_32V_2A()
{
	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
	             INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
	             INA219_CONFIG_SADCRES_12BIT_1S_532US |
	             INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

	ina219.calibrationValue = 4096;
	ina219.currentDivider_mA = 10; // Current LSB = 100uA per bit (1000/100 = 10)
	ina219.powerMultiplier_mW = 2; // Power LSB = 1mW per bit (2/1)

	INA219_setCalibration(ina219.calibrationValue);
	INA219_setConfig(config);
}

void INA219_setCalibration_32V_1A()
{
	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_32V |
	                    INA219_CONFIG_GAIN_8_320MV | INA219_CONFIG_BADCRES_12BIT |
	                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
	                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

	ina219.calibrationValue = 10240;
	ina219.currentDivider_mA = 25;    // Current LSB = 40uA per bit (1000/40 = 25)
	ina219.powerMultiplier_mW = 0.8f; // Power LSB = 800uW per bit

	INA219_setCalibration(ina219.calibrationValue);
	INA219_setConfig(config);
}

void INA219_setCalibration_16V_400mA()
{
	uint16_t config = INA219_CONFIG_BVOLTAGERANGE_16V |
	                    INA219_CONFIG_GAIN_1_40MV | INA219_CONFIG_BADCRES_12BIT |
	                    INA219_CONFIG_SADCRES_12BIT_1S_532US |
	                    INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS;

	ina219.calibrationValue = 8192;
	ina219.currentDivider_mA = 20;    // Current LSB = 50uA per bit (1000/50 = 20)
	ina219.powerMultiplier_mW = 1.0f; // Power LSB = 1mW per bit

	INA219_setCalibration(ina219.calibrationValue);
	INA219_setConfig(config);
}

void INA219_setPowerMode(uint8_t Mode)
{
	uint16_t config = INA219_getConfig();

	switch (Mode) {
		case INA219_CONFIG_MODE_POWERDOWN:
			config = (config & ~INA219_CONFIG_MODE_MASK) | (INA219_CONFIG_MODE_POWERDOWN & INA219_CONFIG_MODE_MASK);
			INA219_setConfig(config);
			break;

		case INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED:
			config = (config & ~INA219_CONFIG_MODE_MASK) | (INA219_CONFIG_MODE_SANDBVOLT_TRIGGERED & INA219_CONFIG_MODE_MASK);
			INA219_setConfig(config);
			break;

		case INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS:
			config = (config & ~INA219_CONFIG_MODE_MASK) | (INA219_CONFIG_MODE_SANDBVOLT_CONTINUOUS & INA219_CONFIG_MODE_MASK);
			INA219_setConfig(config);
			break;

		case INA219_CONFIG_MODE_ADCOFF:
			config = (config & ~INA219_CONFIG_MODE_MASK) | (INA219_CONFIG_MODE_ADCOFF & INA219_CONFIG_MODE_MASK);
			INA219_setConfig(config);
			break;
	}
}

#ifdef _USE_HW_CLI
void cli_ina219(cli_args_t *args)
{
  bool ret = true;
  bool i2c_ret;

  uint32_t i;
  uint32_t pre_time;

  if (args->argc == 1)
  {
    if(args->isStr(0, "scan") == true)
    {
      for (i=0x00; i<= 0x7F; i++)
      {
        if (i2cIsDeviceReady(_DEF_I2C2, i) == true)
        {
          cliPrintf("I2C Addr 0x%X : OK\n", i);
        }
      }
    }
    else if(args->isStr(0, "open") == true)
    {
      i2c_ret = i2cOpen(_DEF_I2C2, 400);
      if (i2c_ret == true)
      {
        cliPrintf("I2C Open OK\n");
      }
      else
      {
        cliPrintf("I2C Open Fail\n");
      }
    }

    if(args->isStr(0, "get_current") == true)
        {
          pre_time = millis();
          uint16_t get_value = INA219_ReadCurrent();

          cliPrintf("ina4725, get current value : %d mA, %d ms\n", (int)get_value, millis()-pre_time);
        }

    if(args->isStr(0, "get_config") == true)
        {
          pre_time = millis();
          uint16_t get_value = INA219_getConfig();

          cliPrintf("ina4725, get current value : %d mA, %d ms\n", (int)get_value, millis()-pre_time);
        }


  }
  else{
	  ret = false;
  }


  if (ret == false)
  {
    cliPrintf( "ina219 open \n");
    cliPrintf( "ina219 scan \n");
    cliPrintf( "ina219 get_current \n");
    cliPrintf( "ina219 get_config \n");
  }
}

#endif


#endif

