/*
 * ap.c
 *
 *  Created on: Dec 6, 2020
 *      Author: baram
 */


#include "ap.h"




void apInit(void)
{
  //cliOpen(_DEF_UART1, 57600);
  uartOpen(_DEF_UART1, 57600);
  uartOpen(_DEF_UART2, 57600);
  i2cOpen(_DEF_I2C1, 400);  // mcp4725,  DAC to current
  i2cOpen(_DEF_I2C2, 400);	// ina219,   current sensor
  spiOpen(_DEF_SPI1);		// max31865, temperature sensor
}

void apMain(void)
{
  uint32_t pre_time;

  uint8_t rx_data;
  uint8_t buf[4];
  uint8_t i =0;
  uint32_t viscosity;

  pre_time = millis();
  while(1)
  {


    if (uartAvailable(_DEF_UART2)>0){
    	rx_data = uartRead(_DEF_UART2);
    	//uartPrintf(_DEF_UART2, "Rx : %d \n", rx_data);

    	buf[i]= rx_data;
    	i++;
    	if(i == 4){
    		viscosity = buf[0];
    		viscosity |= buf[1] <<8;
    		viscosity |= buf[2] <<16;
    		viscosity |= buf[3] <<24;
    		i=0;
    		}
    	}

    if (millis()-pre_time>= 1000){
      pre_time = millis();
      ledToggle(_DEF_LED1);

      uint32_t temperature =4234;
      uint8_t tx_data[4];

      tx_data[0] = (uint8_t)temperature;
      tx_data[1] = (uint8_t)(temperature>>8);
      tx_data[2] = (uint8_t)(temperature>>16);
      tx_data[3] = (uint8_t)(temperature>>24);

      uartWrite(_DEF_UART1, tx_data, 4);
    	}

    }

    cliMain();

}

