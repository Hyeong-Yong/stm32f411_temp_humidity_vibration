/*
 * max31865.h
 *
 *  Created on: 2023. 6. 29.
 *      Author: hwang
 */

#ifndef SRC_HW_DRIVER_SPI_MAX31865_H_
#define SRC_HW_DRIVER_SPI_MAX31865_H_


#include "hw_def.h"
#include <math.h>

#ifdef _USE_HW_MAX31865



//#########################################################################################################################


/*******Configuration Register Definition******************************
 RegisterName     			|     Read address(HEX)            Write address(HEX)
 Configuration    			|	        00h				|			80h
 RTD MSBs  		 		 	|		    01h				|
 RTD LSBs 		  			|  	   	    02h				|
 High Fault Threshold MSB	|			03h				|			83h
 High Fault Threshold LSB	|			04h				|			84h
 Low  Fault Threshold MSB	|			05h				|			85h
 Low  Fault Threshold LSB	|			06h				|			86h
 Fault Status				|			07h				|
*/
#define MAX31865_CONFIG_READ             0x00
#define MAX31865_RTDMSB_READ             0x01
#define MAX31865_RTDLSB_READ  			 0x02
#define MAX31865_HFAULTMSB_READ          0x03
#define MAX31865_HFAULTLSB_READ          0x04
#define MAX31865_LFAULTMSB_READ          0x05
#define MAX31865_LFAULTLSB_READ          0x06
#define MAX31865_FAULTSTAT_READ          0x07

#define MAX31865_CONFIG_WRITE            0x80

//#########################################################################################################################



//#########################################################################################################################

/*******Configuration Register Definition******************************
 [D7] "V_bias"               1:On, 0:OFF
 [D6] "Conversion mode"      1: Auto, 0: OFF
 [D5] "1-shot"               1: 1-shot (auto-clear)
 [D4] "3-wire" 		         1: 3-wire RTD, 0: 2-wire or 4-wire
 [D3] "Fault Detection"      See data-sheet
 [D2] "Fault Detection"
 [D1] "Fault Status Clear"   1: Clear (auto-clear)
 [D0] "50/60Hz filter"       1: 50Hz, 2: 60Hz
*************************************/
#define MAX31865_CONFIG_FILT60HZ        0x00
#define MAX31865_CONFIG_FILT50HZ        0x01
#define MAX31865_CONFIG_FAULTSTAT       0x02
#define MAX31865_CONFIG_3WIRE           0x10
#define MAX31865_CONFIG_1SHOT           0x20
#define MAX31865_CONFIG_AUTO    	    0x40
#define MAX31865_CONFIG_BIAS	        0x80

#define MAX31865_CONFIG_24WIRE          0x00

#define MAX31865_FAULT_HIGHTHRESH       0x80
#define MAX31865_FAULT_LOWTHRESH        0x40
#define MAX31865_FAULT_REFINLOW         0x20
#define MAX31865_FAULT_REFINHIGH        0x10
#define MAX31865_FAULT_RTDINLOW         0x08
#define MAX31865_FAULT_OVUV             0x04

#define RTD_A 3.9083e-3
#define RTD_B -5.775e-7
//#########################################################################################################################


#define MAX31865_RREF      430.0f
#define MAX31865_RNOMINAL  100.0f

//#########################################################################################################################


//#########################################################################################################################
void  max31865_init();
bool  max31865_readTempC(float *readTemp);
bool  max31865_readTempF(float *readTemp);
float max31865_Filter(float	newInput, float	lastOutput, float efectiveFactor);
//#########################################################################################################################

#endif


#endif /* SRC_HW_DRIVER_SPI_MAX31865_H_ */
