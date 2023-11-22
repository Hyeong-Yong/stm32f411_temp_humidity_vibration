/*
 * ap.c
 *
 *  Created on: Dec 6, 2020
 *      Author: baram
 */


#include "ap.h"




void apInit(void)
{
  cliOpen(_DEF_UART1, 57600);
  //uartOpen(_DEF_UART1, 57600);
  uartOpen(_DEF_UART2, 57600);
  i2cOpen(_DEF_I2C1, 400);  // mpu6050,  IMU sensor
  i2cOpen(_DEF_I2C2, 100);	// bme280,   temperature/humidity sensor
  bme280_config(OSRS_2, OSRS_16, OSRS_1, MODE_NORMAL, T_SB_0p5, IIR_16);
  spiOpen(_DEF_SPI1);		// max31865, temperature sensor
}

void apMain(void)
{
  uint32_t pre_time;


  pre_time = millis();
  while(1)
  {

    if (uartAvailable(_DEF_UART2)>0){
    	//rx_data = uartRead(_DEF_UART2);
    	//uartPrintf(_DEF_UART2, "Rx : %d \n", rx_data);
    	}

    if (millis()-pre_time>= 1000){
      pre_time = millis();
      ledToggle(_DEF_LED1);
    }



    cliMain();
  }
}

