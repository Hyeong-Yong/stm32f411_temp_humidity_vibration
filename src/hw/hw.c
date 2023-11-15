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

  i2cInit();
  mcp4725_init();
  INA219_Init();

  spiInit();
  max31865_init();
}
