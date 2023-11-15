/*
 * mcp4725.c
 *
 *  Created on: 2023. 5. 26.
 *      Author: hwang
 */

/***************************************************************************************************/
/*
 *
   This is a HAL based library for MCP4725, 12-bit Digital-to-Analog Converter with EEPROM

   NOTE:
   - operating/reference voltage 2.7v - 5.5v
   - add 100μF & 0.1 μF bypass capacitors within 4mm to Vdd
   - output voltage from 0 to operating voltage
   - maximum output current 25mA
   - output impedance 1 Ohm
   - maximum output load 1000pF/0.001μF in parallel with 5 kOhm
   - voltage settling time 6 μsec - 10 μsec
   - slew rate 0.55 V/μs
   - device has 14-bit EEPROM with on-chip charge pump circuit for fail-safe writing
   - estimated EEPROM endurance 1 million write cycles
   - if Vdd < 2v all circuits & output disabled, when Vdd
     increases above Vpor device takes a reset state & upload data from EEPROM

   ported by : Salman Motlaq
	 sourse code: https://github.com/SMotlaq

	 from an Arduino lib written by : enjoyneering79
   sourse code: https://github.com/enjoyneering

   GNU GPL license, all text above must be included in any redistribution,
   see link for details  - https://www.gnu.org/licenses/licenses.html
*/
/***************************************************************************************************/



#include "mcp4725.h"

#ifdef _USE_HW_MCP4725

typedef struct MCP
{
	// Privates:
  I2C_HandleTypeDef *hi2c;
  MCP4725Ax_ADDRESS i2cAddress;
  float             refVoltage;
  uint16_t          bitsPerVolt;
} MCP4725;

MCP4725 mcp4725;

extern I2C_HandleTypeDef hi2c2;
static bool mcp4725_writeComand(MCP4725* mcp4725, uint16_t value, MCP4725_COMMAND_TYPE mode, MCP4725_POWER_DOWN_TYPE powerType);
static uint16_t mcp4725_readRegister(MCP4725* mcp4725, MCP4725_READ_TYPE dataType);
static uint8_t	mcp4725_getEepromBusyFlag();


static bool is_init = false;
const uint8_t i2c_ch = _DEF_I2C1; //channel 0: _DEF_I2C2;

bool mcp4725_init(void){
	bool ret = true;

	mcp4725.hi2c = &hi2c2;
	mcp4725.i2cAddress = MCP4725A0_ADDR_A00;
	mcp4725.refVoltage = 5.0;
	mcp4725_setReferenceVoltage(mcp4725.refVoltage); //set _refVoltage & _bitsPerVolt variables
	is_init = true;

	uint8_t mcp4725_isReady = i2cIsDeviceReady(i2c_ch, mcp4725.i2cAddress);
	if(mcp4725_isReady == HAL_OK)
		{
			ret = false;
		}

	#ifdef _USE_HW_CLI
		cliAdd("mcp4725", cli_mcp4725);
	#endif

	return ret;
}


bool mcp4725_IsInit(void)
{
  return is_init;
}

/**************************************************************************/
/*
    MCP4725_isConnected()

    Check the connection
*/
/**************************************************************************/
bool mcp4725_isConnected()
{
	bool ret = false;
	ret = i2cIsDeviceReady(i2c_ch, mcp4725.i2cAddress);
	return ret;
}


/**************************************************************************/
/*
    setReferenceVoltage()

    Set reference voltage
*/
/**************************************************************************/
void mcp4725_setReferenceVoltage(float value)
{
   if   (value == 0) mcp4725.refVoltage = MCP4725_REFERENCE_VOLTAGE; //sanity check, avoid division by zero
   else              mcp4725.refVoltage = value;

   mcp4725.bitsPerVolt = (float)MCP4725_STEPS / mcp4725.refVoltage;         //TODO: check accuracy with +0.5
}

/**************************************************************************/
/*
    getReferenceVoltage()

    Return reference voltage
*/
/**************************************************************************/
float mcp4725_getReferenceVoltage(void)
{
  return mcp4725.refVoltage;
}






/**************************************************************************/
/*
    writeComand()

    Writes value to DAC register or EEPROM

    NOTE:
    - "MCP4725_FAST_MODE" bit format:
      15    14    13   12   11   10   9   8   7   6   5   4   3   2   1   0-bit
      C2=0, C1=0, PD1, PD0, D11, D10, D9, D8, D7, D6, D5, D4, D3, D2, D1, D0
    - "MCP4725_REGISTER_MODE" bit format:
      23    22    21    20   19   18   17   16  15   14   13  12  11  10  9   8   7   6    5   4  3   2   1   0-bit
      C2=0, C1=1, C0=0, xx,  xx,  PD1, PD0, xx, D11, D10, D9, D8, D7, D6, D5, D4, D3, D2, D1, D0, xx, xx, xx, xx
    - "MCP4725_EEPROM_MODE" bit format:
      23    22    21    20   19   18   17   16  15   14   13  12  11  10  9   8   7   6    5   4  3   2   1   0-bit
      C2=0, C1=1, C0=1, xx,  xx,  PD1, PD0, xx, D11, D10, D9, D8, D7, D6, D5, D4, D3, D2, D1, D0, xx, xx, xx, xx

    - "MCP4725_POWER_DOWN_OFF"
      PD1 PD0
      0,  0
    - "MCP4725_POWER_DOWN_1KOHM"
      PD1 PD0
      0,  1
    - "MCP4725_POWER_DOWN_100KOHM"
      1,  0
    - "MCP4725_POWER_DOWN_500KOHM"
      1,  1
*/

//mcp4725_writeComand(mcp4725, 340, MCP4725_FAST_MODE, MCP4725_POWER_DOWN_OFF)
/**************************************************************************/
bool mcp4725_writeComand(MCP4725* mcp4725, uint16_t value, MCP4725_COMMAND_TYPE mode, MCP4725_POWER_DOWN_TYPE powerType)
{
	bool ret = false;
	uint8_t buffer[3];
  //Wire.beginTransmission(_i2cAddress);

  switch (mode)
  {
    case MCP4725_FAST_MODE:                                            //see MCP4725 datasheet on p.18

      //Wire.send(mode | (powerType << 4)  | highByte(value));
      //Wire.send(lowByte(value));

			buffer[0] = mode | (powerType << 4)  | highByte(value);
			buffer[1] = lowByte(value);
			ret = i2cWriteData(i2c_ch, mcp4725->i2cAddress, buffer, 2, 1000);
			//I2C_Stat = HAL_I2C_Master_Transmit(mcp4725->hi2c, mcp4725->i2cAddress, buffer, 2, 1000);

      break;

    case MCP4725_REGISTER_MODE: case MCP4725_EEPROM_MODE:              //see MCP4725 datasheet on p.19
      value = value << 4;                                              //D11,D10,D9,D8,D7,D6,D5,D4,  D3,D2,D1,D0,xx,xx,xx,xx
      //Wire.send(mode  | (powerType << 1));
      //Wire.send(highByte(value));
      //Wire.send(lowByte(value));

			buffer[0] = mode  | (powerType << 1);
			buffer[1] = highByte(value);
			buffer[2] = lowByte(value);
			ret = i2cWriteData(i2c_ch, mcp4725->i2cAddress, buffer, 3, 100);

			break;
  }

  if (ret == false) return false;                   //send data over i2c & check for collision on i2c bus

  if (mode == MCP4725_EEPROM_MODE)
  {
    if (mcp4725_getEepromBusyFlag(mcp4725) == 1) return true;                      //write completed, success!!!
                                     HAL_Delay(MCP4725_EEPROM_WRITE_TIME); //typical EEPROM write time 25 msec
    if (mcp4725_getEepromBusyFlag(mcp4725) == 1) return true;                      //write completed, success!!!
                                     HAL_Delay(MCP4725_EEPROM_WRITE_TIME); //maximum EEPROM write time 25 + 25 = 50 msec
  }

  return true;                                                         //success!!!
}

/**************************************************************************/
/*
    readRegister()

    Read DAC register via i2c bus

    NOTE:
    - read output bit format:
      39  38  37 36 35 34  33  32  31  30  29 28 27 26 25 24  23 22 21 20 19 18 17 16  15 14  13  12 11  10  9  8   7  6  5  4  3  2  1  0-bit
      BSY,POR,xx,xx,xx,PD1,PD0,xx, D11,D10,D9,D8,D7,D6,D5,D4, D3,D2,D1,D0,xx,xx,xx,xx, xx,PD1,PD0,xx,D11,D10,D9,D8, D7,D6,D5,D4,D3,D2,D1,D0
      ------ Settings data ------  ---------------- DAC register data ---------------  ------------------- EEPROM data --------------------
    - see MCP4725 datasheet on p.20
*/
/**************************************************************************/
uint16_t mcp4725_readRegister(MCP4725* mcp4725, MCP4725_READ_TYPE dataType)
{
	uint16_t value = dataType;                             //convert enum to integer to avoid compiler warnings
	uint16_t ret_val = 0 ;
	uint8_t buffer[dataType];
	bool ret=false;
	ret = i2cReadData(i2c_ch, mcp4725->i2cAddress, buffer, dataType, 100);
//	I2C_Stat = HAL_I2C_Master_Receive(_MCP4725->hi2c, _MCP4725->_i2cAddress, buffer, dataType, 1000);

  if (ret == false) return MCP4725_ERROR;

  /* read data from buffer */
  switch (dataType)
  {
    case MCP4725_READ_SETTINGS:
      ret_val = buffer[0];

      break;

    case MCP4725_READ_DAC_REG: case MCP4725_READ_EEPROM:

      ret_val = buffer[value-2];
      ret_val = (ret_val << 8) | buffer[value-1];
      break;
  }

  return ret_val;
}






/**************************************************************************/
/*
    setValue()

    Set output voltage to a fraction of Vref

    NOTE:
    -  mode:
      - "MCP4725_FAST_MODE"...........writes 2-bytes, data & power type to
                                      DAC register & EEPROM is not affected
      - "MCP4725_REGISTER_MODE".......writes 3-bytes, data & power type to
                                      DAC register & EEPROM is not affected
      - "MCP4725_EEPROM_MODE".........writes 3-bytes, data & power type to
                                      DAC register & EEPROM
    - powerType:
      - "MCP4725_POWER_DOWN_OFF"......power down off, draws 0.40mA no load
                                      & 0.29mA maximum load
      - "MCP4725_POWER_DOWN_1KOHM"....power down on with 1 kOhm to ground,
                                      draws 60nA
      - "MCP4725_POWER_DOWN_100KOHM"..power down on with 100 kOhm to ground
      - "MCP4725_POWER_DOWN_500KOHM"..power down on with 500kOhm to ground
*/
/**************************************************************************/
bool mcp4725_setValue(uint16_t value, MCP4725_COMMAND_TYPE mode, MCP4725_POWER_DOWN_TYPE powerType)
{
  #ifndef MCP4725_DISABLE_SANITY_CHECK
  if (value > MCP4725_MAX_VALUE) value = MCP4725_MAX_VALUE; //make sure value never exceeds threshold
  #endif

  return mcp4725_writeComand(&mcp4725, value, mode, powerType);
}

/**************************************************************************/
/*
    setVoltage()

    Set output voltage to a fraction of Vref
*/
/**************************************************************************/
bool mcp4725_setVoltage(float voltage, MCP4725_COMMAND_TYPE mode, MCP4725_POWER_DOWN_TYPE powerType)
{
  uint16_t value = 0;

  /* convert voltage to DAC bits */
  #ifndef MCP4725_DISABLE_SANITY_CHECK
  if      (voltage >= mcp4725->refVoltage) value = MCP4725_MAX_VALUE;      					 //make sure value never exceeds threshold
  else if (voltage <= 0)					           value = 0;
  else                            					 value = voltage * mcp4725->bitsPerVolt; //xx,xx,xx,xx,D11,D10,D9,D8 ,D7,D6,D4,D3,D2,D9,D1,D0
  #else
  value = voltage * mcp4725.bitsPerVolt;                                											 //xx,xx,xx,xx,D11,D10,D9,D8 ,D7,D6,D4,D3,D2,D9,D1,D0
  #endif

  return mcp4725_writeComand(&mcp4725, value, mode, powerType);
}

/**************************************************************************/
/*
    getValue()

    Read current DAC value from DAC register

    NOTE:
    - see MCP4725 datasheet on p.20
*/
/**************************************************************************/
uint16_t mcp4725_getValue()
{
  uint16_t value = mcp4725_readRegister(&mcp4725, MCP4725_READ_DAC_REG); //D11,D10,D9,D8,D7,D6,D5,D4, D3,D2,D1,D0,xx,xx,xx,xx

  if (value != MCP4725_ERROR) {return value >> 4;}       //00,00,00,00,D11,D10,D9,D8,  D7,D6,D5,D4,D3,D2,D1,D0
                              return value;            //collision on i2c bus
}

/**************************************************************************/
/*
    getVoltage()

    Read current DAC value from DAC register & convert to voltage
*/
/**************************************************************************/
float mcp4725_getVoltage()
{
  float value = mcp4725_getValue(&mcp4725);

  if (value != MCP4725_ERROR) {return value / mcp4725.bitsPerVolt;}
                              return value;
}

/**************************************************************************/
/*
    getStoredValue()

    Read DAC value from EEPROM

    NOTE:
    - see MCP4725 datasheet on p.20
*/
/**************************************************************************/
uint16_t mcp4725_getStoredValue()
{
  uint16_t value = mcp4725_readRegister(&mcp4725, MCP4725_READ_EEPROM); //xx,PD1,PD0,xx,D11,D10,D9,D8, D7,D6,D5,D4,D3,D2,D1,D0

  if (value != MCP4725_ERROR) {return value & 0x0FFF;}  //00,00,00,00,D11,D10,D9,D8,   D7,D6,D5,D4,D3,D2,D1,D0
                              return value;           //collision on i2c bus
}

/**************************************************************************/
/*
    getStoredVoltage()

    Read stored DAC value from EEPROM & convert to voltage
*/
/**************************************************************************/
float mcp4725_getStoredVoltage()
{
  float value = mcp4725_getStoredValue(&mcp4725);

  if (value != MCP4725_ERROR) {return value / mcp4725.bitsPerVolt;}
                              return value;
}

/**************************************************************************/
/*
    getPowerType()

    Return current power type from DAC register

    NOTE:
    - "MCP4725_POWER_DOWN_OFF"
      PD1 PD0
      0,  0
    - "MCP4725_POWER_DOWN_1KOHM"
      PD1 PD0
      0,  1
    - "MCP4725_POWER_DOWN_100KOHM"
      1,  0
    - "MCP4725_POWER_DOWN_500KOHM"
      1,  1
    - in the power-down modes Vout is off
    - see MCP4725 datasheet on p.15
    - see MCP4725 datasheet on p.20
*/
/**************************************************************************/
uint16_t mcp4725_getPowerType()
{
  uint16_t value = mcp4725_readRegister(&mcp4725, MCP4725_READ_SETTINGS); //BSY,POR,xx,xx,xx,PD1,PD0,xx

  if (value != MCP4725_ERROR)
  {
           value &= 0x0006;                             //00,00,00,00,00,PD1,PD0,00
    return value >> 1;                                  //00,00,00,00,00,00,PD1,PD0
  }

  return value;                                         //collision on i2c bus
}

/**************************************************************************/
/*
    getStoredPowerType()

    Return stored power type from EEPROM

    NOTE:
    - "MCP4725_POWER_DOWN_OFF"
      PD1 PD0
      0,  0
    - "MCP4725_POWER_DOWN_1KOHM"
      PD1 PD0
      0,  1
    - "MCP4725_POWER_DOWN_100KOHM"
      1,  0
    - "MCP4725_POWER_DOWN_500KOHM"
      1,  1
    - see MCP4725 datasheet on p.20
*/
/**************************************************************************/
uint16_t mcp4725_getStoredPowerType()
{
  uint16_t value = mcp4725_readRegister(&mcp4725, MCP4725_READ_EEPROM); //xx,PD1,PD0,xx,D11,D10,D9,D8,  D7,D6,D5,D4,D3,D2,D1,D0

  if (value != MCP4725_ERROR)
  {
    value = value << 1;                               //PD1,PD0,xx,D11,D10,D9,D8,D7  D6,D5,D4,D3,D2,D1,D0,00
    return  value >> 14;                              //00,00,00,00,00,00,00,00      00,00,00,00,00,00,PD1,PD0
  }

  return value;                                       //collision on i2c bus
}





/**************************************************************************/
/*
    getEepromBusyFlag()

    Return EEPROM writing status from DAC register

    NOTE:
    - any new write command including repeat bytes during EEPROM write mode
      is ignored
    - see MCP4725 datasheet on p.20
*/
/**************************************************************************/
uint8_t mcp4725_getEepromBusyFlag()
{
  uint16_t value = mcp4725_readRegister(&mcp4725, MCP4725_READ_SETTINGS); //BSY,POR,xx,xx,xx,PD1,PD0,xx

  if (value != MCP4725_ERROR) {
	  return (value & 0x80)==0x80;		//1 - completed, 0 - incompleted
  }
                              return 0;										//collision on i2c bus
}





/**************************************************************************/
/*
    reset()

    Reset MCP4725 & upload data from EEPROM to DAC register

    NOTE:
    - use with caution, "general call" command may affect all slaves
      on i2c bus
    - if Vdd < 2v all circuits & output disabled, when the Vdd
      increases above Vpor device takes a reset state
*/
/**************************************************************************/
bool mcp4725_reset()
{
	bool ret = false;
	// First Byte : General Call Address
	// Second Byte : General Call Reset

	uint8_t buffer[1] = {MCP4725_GENERAL_CALL_RESET};
	ret = i2cWriteData(i2c_ch, MCP4725_GENERAL_CALL_ADDRESS, buffer, 1, 100);
	return ret;
}

/**************************************************************************/
/*
    wakeUP()

    Wake up & upload value from DAC register

    NOTE:
    - use with caution, "general call" command may affect all slaves
      on i2c bus
    - resets current power-down bits, EEPROM power-down bit are
      not affected
*/
/**************************************************************************/
bool mcp4725_WakeUP()
{
	bool ret= false;
	// First Byte : General Call Address
	// Second Byte : General Call Wake-up
	uint8_t buffer[1] = {MCP4725_GENERAL_WAKE_UP};
	ret = i2cWriteData(i2c_ch, MCP4725_GENERAL_CALL_ADDRESS, buffer, 1, 100);

	return ret;
}



#ifdef _USE_HW_CLI
void cli_mcp4725(cli_args_t *args)
{
  bool ret = true;
  bool i2c_ret;
  bool mcp4725_ret;

  uint32_t i;
  uint32_t pre_time;


  if (args->argc == 1)
  {


    if(args->isStr(0, "scan") == true)
    {
      for (i=0x00; i<= 0x7F; i++)
      {
        if (i2cIsDeviceReady(0, i) == true)
        {
          cliPrintf("I2C Addr 0x%X : OK\n", i);
        }
      }
    }

    else if(args->isStr(0, "open") == true)
    {
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

    if(args->isStr(0, "get_value") == true)
        {
          pre_time = millis();
          uint16_t get_value = mcp4725_getValue();


		  if (get_value != MCP4725_ERROR)
		  {
			  mcp4725_ret = true;
		  }
		  else{
			  mcp4725_ret = false;
		  }

          if (mcp4725_ret == true)
          {
            cliPrintf("mcp4725, get value : %d, %d ms\n", (int)get_value, millis()-pre_time);
          }
          else
          {
            cliPrintf(" DAC-mcp4725 - Fail \n");
          }
        }
  }

  else if (args->argc == 2)
  {


    float value = args->getData(1);

    if(args->isStr(0, "set_value") == true)
    {
      pre_time = millis();

      mcp4725_ret = mcp4725_setValue(value, MCP4725_FAST_MODE, MCP4725_POWER_DOWN_OFF);


      if (mcp4725_ret == true)
      {
        cliPrintf("mcp4725, DAC value : %d, %d ms\n", (int)value, millis()-pre_time);
      }
      else
      {
        cliPrintf(" DAC-mcp4725 - Fail \n");
      }
    }
    else
    {
      ret = false;
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
    cliPrintf( "mcp4725 get_value \n");
    cliPrintf( "mcp4725 set_value DAC_value \n");
  }
}

#endif


#endif
