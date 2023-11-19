/*
 * hw.h
 *
 *  Created on: Dec 6, 2020
 *      Author: baram
 */

#ifndef SRC_HW_HW_H_
#define SRC_HW_HW_H_


#include "hw_def.h"


#include "led.h"
#include "uart.h"
#include "usb.h"
#include "rtc.h"
#include "reset.h"
#include "flash.h"
#include "cli.h"
#include "button.h"
#include "gpio.h"

#include "i2c.h"
#include "mpu6050.h"
#include "bme280.h"
#include "ina219.h"
//#include "mcp4725.h"

#include "spi.h"
//#include "max31865.h"

void hwInit(void);


#endif /* SRC_HW_HW_H_ */
