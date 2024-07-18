/*
 * hw.c
 *
 *  Created on: Dec 6, 2020
 *      Author: baram
 */

#include "hw.h"

void hwInit(void)
{
  bspInit();

  cliInit();
  ledInit();
  usbInit();
  uartInit();
  buttonInit();
  gpioInit();
  spiInit();

  //i2cInit();
  //mpu6050_init();

  //bme280_init();

  //  max31865_init();
}
