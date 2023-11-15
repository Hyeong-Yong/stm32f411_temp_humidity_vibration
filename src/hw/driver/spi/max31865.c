/*
 * max31865.c
 *
 *  Created on: 2023. 6. 29.
 *      Author: hwang
 */

#include "max31865.h"
#include "spi.h"
#include "gpio.h"

#include "cli.h"

#ifdef _USE_HW_MAX31865

#ifdef _USE_HW_CLI
static void cli_max31865(cli_args_t* args);
#endif

#define _SPI_CH	_DEF_SPI1
#define _GPIO_MAX31865_CS _DEF_GPIO2

static uint8_t* max31865_readRegisterN(uint8_t addr, uint8_t n);
static uint8_t  max31865_readRegister8(uint8_t addr);
static uint16_t max31865_readRegister16(uint8_t addr);
static uint8_t  max31865_readFault();
static void max31865_clearFault();
static void max31865_enableBias(uint8_t enable);
static void max31865_autoConvert(uint8_t enable);
static void max31865_setWires(uint8_t numWires);
static void max31865_setFilter(uint8_t filterHz);


typedef struct
{
  SPI_HandleTypeDef *spi;
  uint8_t           lock;
}MAX31865_t;


MAX31865_t  pt100;


extern SPI_HandleTypeDef hspi1; // MAX31865

//cs : gpio 핀 추가

void max31865_init() //uint8_t  numwires, uint8_t filterHz
{
  pt100.spi = &hspi1;
  pt100.lock = 1;

// CS핀 OFF
  gpioPinWrite(_SPI_CH, _DEF_LOW);

// 딜레이
  delay(100);

// Setting configuration
  max31865_setWires(MAX31865_CONFIG_3WIRE);
  max31865_enableBias(MAX31865_CONFIG_BIAS);
  max31865_autoConvert(MAX31865_CONFIG_AUTO);
  max31865_clearFault();
  max31865_setFilter(MAX31865_CONFIG_FILT60HZ);

#ifdef _USE_HW_CLI
  cliAdd("max31865", cli_max31865);

#endif

}


uint8_t* max31865_readRegisterN(uint8_t addr, uint8_t n)
{
	uint8_t dummy = 0xFF;
	addr &= 0x7F; ////[A7]=0 (0111 1111) : read

	uint8_t *ret_buf ={0};
	uint8_t i=0;

	gpioPinWrite(_GPIO_MAX31865_CS, _DEF_HIGH);
	spiTx(_SPI_CH, &addr, 1);

	while (n--)
	{
		ret_buf[i] = spiTransfer8(_SPI_CH, dummy);
		i++;
	}
	gpioPinWrite(_GPIO_MAX31865_CS, _DEF_HIGH);

	return ret_buf;
}

uint8_t max31865_readRegister8(uint8_t addr)
{
	uint8_t* ret_buf;
	ret_buf = max31865_readRegisterN(addr, 1);

	uint8_t ret= ret_buf[0];
	return ret;
}

uint16_t max31865_readRegister16(uint8_t addr)
{
	uint8_t *ret_buf;
	ret_buf = max31865_readRegisterN(addr, 2);

	uint16_t ret = ret_buf[0];
	ret <<= 8;
	ret |=  ret_buf[1];
	return ret;
}

void max31865_writeRegister8(uint8_t addr, uint8_t data)
{
	gpioPinWrite(_SPI_CH, _DEF_HIGH);
	addr |= 0x80; //[A7]=1 (1000 0000) : write
	spiTx(_SPI_CH, &addr, 1);
	spiTx(_SPI_CH, &data, 1);
	gpioPinWrite(_SPI_CH, _DEF_LOW);
}

uint8_t max31865_readFault()
{
  return max31865_readRegister8(MAX31865_FAULTSTAT_READ);
}

void max31865_clearFault()
{
	uint8_t t = max31865_readRegister8(MAX31865_CONFIG_READ);
	t &= ~0x2C;
	t |= MAX31865_CONFIG_FAULTSTAT;
	max31865_writeRegister8(MAX31865_CONFIG_READ, t);
}

void max31865_enableBias(uint8_t enable)
{
	uint8_t t = max31865_readRegister8(MAX31865_CONFIG_READ);
	if (enable)
		t |= MAX31865_CONFIG_BIAS;
	else
		t &= ~MAX31865_CONFIG_BIAS;
	max31865_writeRegister8(MAX31865_CONFIG_READ, t);
}

void max31865_autoConvert(uint8_t enable)
{
	uint8_t t = max31865_readRegister8(MAX31865_CONFIG_READ);
	if (MAX31865_CONFIG_AUTO)
		t |= MAX31865_CONFIG_AUTO;
	else
		t &= ~MAX31865_CONFIG_AUTO;
	max31865_writeRegister8(MAX31865_CONFIG_READ, t);
}

void max31865_setWires(uint8_t numWires)
{
	uint8_t t = max31865_readRegister8(MAX31865_CONFIG_READ);
	if (numWires == MAX31865_CONFIG_3WIRE)
		t |= MAX31865_CONFIG_3WIRE;
	else
		t &= ~MAX31865_CONFIG_3WIRE;
	max31865_writeRegister8(MAX31865_CONFIG_READ, t);
}

void max31865_setFilter(uint8_t filterHz)
{
	uint8_t t = max31865_readRegister8(MAX31865_CONFIG_READ);
	if (filterHz == MAX31865_CONFIG_FILT50HZ)
		t |= MAX31865_CONFIG_FILT50HZ;
	else
		t &= ~MAX31865_CONFIG_FILT50HZ;
	max31865_writeRegister8(MAX31865_CONFIG_READ, t);
}

uint16_t max31865_readRTD()
{
	max31865_clearFault();
	max31865_enableBias(1);
	delay(10);
	uint8_t t = max31865_readRegister8(MAX31865_CONFIG_READ);
	t |= MAX31865_CONFIG_1SHOT;
	max31865_writeRegister8(MAX31865_CONFIG_READ, t);
	delay(65);
	uint16_t rtd = max31865_readRegister16(MAX31865_RTDMSB_READ);
	rtd >>= 1;
	return rtd;
}


bool max31865_readTempC(float *readTemp)
{
  if(pt100.lock == 1)
    {delay(1);}
  pt100.lock = 1;
  bool isOk = false;

  float Z1, Z2, Z3, Z4, Rt, temp;
	Rt = max31865_readRTD();
	Rt /= 32768;
	Rt *= MAX31865_RREF;
	Z1 = -RTD_A;
	Z2 = RTD_A * RTD_A - (4 * RTD_B);
	Z3 = (4 * RTD_B) / MAX31865_RNOMINAL;
	Z4 = 2 * RTD_B;
	temp = Z2 + (Z3 * Rt);
	temp = (sqrtf(temp) + Z1) / Z4;

	if (temp >= 0)
  {
    *readTemp = temp;
    if(max31865_readFault() == 0)
      isOk = true;
    pt100.lock = 0;
    return isOk;
  }
	Rt /= MAX31865_RNOMINAL;
	Rt *= 100;
	float rpoly = Rt;
	temp = -242.02;
	temp += 2.2228 * rpoly;
	rpoly *= Rt;  // square
	temp += 2.5859e-3 * rpoly;
	rpoly *= Rt;  // ^3
	temp -= 4.8260e-6 * rpoly;
	rpoly *= Rt;  // ^4
	temp -= 2.8183e-8 * rpoly;
	rpoly *= Rt;  // ^5
	temp += 1.5243e-10 * rpoly;

  *readTemp = temp;
  if(max31865_readFault() == 0)
    isOk = true;
  pt100.lock = 0;
  return isOk;
}

bool max31865_readTempF(float *readTemp)
{
  bool isOk = max31865_readTempC(readTemp);
  *readTemp = (*readTemp * 9.0f / 5.0f) + 32.0f;
  return isOk;
}

float max31865_Filter(float	newInput, float	lastOutput, float efectiveFactor)
{
	return ((float)lastOutput*(1.0f-efectiveFactor)) + ((float)newInput*efectiveFactor) ;
}

void cli_max31865(cli_args_t* args)
{
	  bool ret = true;
	  bool spi_ret;


	  uint32_t pre_time;

	  if (args->argc == 1)
	  {
	    if(args->isStr(0, "open") == true)
	    {
	      spi_ret = spiOpen(_SPI_CH);
	      if (spi_ret == true)
	      {
	        cliPrintf("SPI Open OK\n");
	      }
	      else
	      {
	        cliPrintf("SPI Open Fail\n");
	      }
	    }
	    if(args->isStr(0, "get_temperature") == true)
	        {
	          pre_time = millis();
	          float temperature = 10.1;
	          float* p_temperature = &temperature;
	          spi_ret = max31865_readTempC(p_temperature);
	          if (spi_ret == true)
	          {
		          cliPrintf("max31865, temperature : %.2f, %d ms\n", temperature, millis()-pre_time);
	          }
	          else
	          {
	        	cliPrintf("temperature Read fail\n");
	          }
	        }

	  }
	  else{
		  ret = false;
	  }


	  if (ret == false)
	  {
	    cliPrintf( "max31865 open \n");
	    cliPrintf( "max31865 get_temperature \n");
	  }

}


#endif
