/*
 * bme280.h
 *
 *  Created on: 2023. 11. 17.
 *      Author: User
 */

#ifndef SRC_HW_DRIVER_I2C_BME280_H_
#define SRC_HW_DRIVER_I2C_BME280_H_

#include "hw.h"

#define BME280_ADDRESS 0x76  // SDIO is grounded, the 7 bit address is 0x76 and 8 bit address = 0x76<<1 = 0xEC

//#define SUPPORT_64BIT 1
#define SUPPORT_32BIT 1


// Oversampling definitions
#define OSRS_OFF    	0x00
#define OSRS_1      	0x01
#define OSRS_2      	0x02
#define OSRS_4      	0x03
#define OSRS_8      	0x04
#define OSRS_16     	0x05

// MODE Definitions
#define MODE_SLEEP      0x00
#define MODE_FORCED     0x01
#define MODE_NORMAL     0x03

// Standby Time
#define T_SB_0p5    	0x00
#define T_SB_62p5   	0x01
#define T_SB_125    	0x02
#define T_SB_250    	0x03
#define T_SB_500    	0x04
#define T_SB_1000   	0x05
#define T_SB_10     	0x06
#define T_SB_20     	0x07

// IIR Filter Coefficients
#define IIR_OFF     	0x00
#define IIR_2       	0x01
#define IIR_4       	0x02
#define IIR_8       	0x03
#define IIR_16      	0x04

// REGISTERS DEFINITIONS
#define ID_REG      	0xD0
#define RESET_REG  		0xE0
#define CTRL_HUM_REG    0xF2
#define STATUS_REG      0xF3
#define CTRL_MEAS_REG   0xF4
#define CONFIG_REG      0xF5
#define PRESS_MSB_REG   0xF7


/* Configuration for the BME280
 * @osrs is the oversampling to improve the accuracy
 *       if osrs is set to OSRS_OFF, the respective measurement will be skipped
 *       It can be set to OSRS_1, OSRS_2, OSRS_4, etc. Check the header file
 *
 * @mode can be used to set the mode for the device
 *       MODE_SLEEP will put the device in sleep
 *       MODE_FORCED device goes back to sleep after one measurement. You need to use the BME280_WakeUP() function before every measurement
 *       MODE_NORMAL device performs measurement in the normal mode. Check datasheet page no 16
 *
 * @t_sb is the standby time. The time sensor waits before performing another measurement
 *       It is used along with the normal mode. Check datasheet page no 16 and page no 30
 *
 * @filter is the IIR filter coefficients
 *         IIR is used to avoid the short term fluctuations
 *         Check datasheet page no 18 and page no 30
 */



bool bme280_init();
int bme280_config(uint8_t osrs_t, uint8_t osrs_p, uint8_t osrs_h, uint8_t mode, uint8_t t_sb, uint8_t filter);
void trimRead(void);
void mbe280_wakeUp(void);
void mbe280_measure(void);


#endif /* SRC_HW_DRIVER_I2C_BME280_H_ */
