/*
 * mpu6050.h
 *
 *  Created on: 2023. 11. 17.
 *      Author: User
 */

#ifndef SRC_HW_DRIVER_I2C_MPU6050_H_
#define SRC_HW_DRIVER_I2C_MPU6050_H_

#include "hw.h"
#include "i2c.h"

#ifdef _USE_HW_MPU6050

#define MPU6050_ADDR 0xD0

#define SMPLRT_DIV_REG 0x19
#define GYRO_CONFIG_REG 0x1B
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define TEMP_OUT_H_REG 0x41
#define GYRO_XOUT_H_REG 0x43
#define PWR_MGMT_1_REG 0x6B
#define WHO_AM_I_REG 0x75


bool mpu6050_init(void);

#endif

#endif /* SRC_HW_DRIVER_I2C_MPU6050_H_ */
