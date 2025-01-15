
/**
 ******************************************************************************
 * @file    adxl345.h
 * @author  - Anthony E.Raterta
 * @version V1.0.0
 * @date    12-Jan-2025
 * @brief   Contains all the prototypes for the adxl345.C
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2024 mcu-dev
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
#ifndef ADXL345_H
#define ADXL345_H

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "utils/i2c.h"

/*******************************STATUSES***************************************/
typedef enum {
  ADXL345_STATUS_SUCCESS   = 0,
  ADXL345_STATUS_API_ERR   = -1,
  ADXL345_STATUS_INPUT_ERR = -2,
  ADXL345_STATUS_INIT_ERR  = -3,
} ADXL345_RETURN_STATUS;

/*****************************ID REGISTERS*************************************/

#define ADXL345_I2C_ADDRESS 0x53
#define ADXL345_DEV_ID      0X0E

/*********************************MASKS****************************************/
/*********************************REGISTERS************************************/

#define ADXL345_REG_DEV_ID      0x00
#define ADXL345_REG_BW_RATE     0x2C
#define ADXL345_REG_POWER_CTL   0x2D
#define ADXL345_REG_INT_ENABLE  0x2E
#define ADXL345_REG_INT_MAP     0x2F
#define ADXL345_REG_INT_SOURCE  0x30
#define ADXL345_REG_DATAX0      0x32
#define ADXL345_REG_DATAX1      0x33
#define ADXL345_REG_DATAY0      0x34
#define ADXL345_REG_DATAY1      0x35
#define ADXL345_REG_DATAZ0      0x36
#define ADXL345_REG_DATAZ1      0x37
#define ADXL345_REG_FIFO_CTL    0x38
#define ADXL345_REG_FIFO_STATUS 0x39

/*****************************ACC DESCRIPTORS**********************************/
typedef enum {
  ADXL345_NORMAL_MODE    = 0x00,
  ADXL345_LOW_POWER_MODE = 0x01
} adxl345_power_mode;

typedef enum {
  ADXL345_ODR_0_10HZ = 0x00,
  ADXL345_ODR_0_20HZ = 0x00,
  ADXL345_ODR_0_39HZ = 0x00,
  ADXL345_ODR_0_78HZ = 0x00,
  ADXL345_ODR_1_56HZ = 0x00,
  ADXL345_ODR_3_13HZ = 0x00,
  ADXL345_ODR_6_25HZ = 0x00,
  ADXL345_ODR_12_5HZ = 0x00,
  ADXL345_ODR_25HZ   = 0x00,
  ADXL345_ODR_50HZ   = 0x00,
  ADXL345_ODR_100HZ  = 0x00,
  ADXL345_ODR_200HZ  = 0x00,
  ADXL345_ODR_400HZ  = 0x00,
  ADXL345_ODR_800HZ  = 0x00,
  ADXL345_ODR_1K6HZ  = 0x00,
  ADXL345_ODR_3K2HZ  = 0x00,
} adxl345_odr;

typedef enum {
  ADXL345_SCALE_2G  = 0x00,
  ADXL345_SCALE_4G  = 0x01,
  ADXL345_SCALE_8G  = 0x02,
  ADXL345_SCALE_16G = 0x03,

} adxl345_scale;

typedef enum {
  ADXL345_RES_10BIT = 0x00,
  ADXL345_RES_16BIT = 0x01,
} adxl345_resolution;

/**********************************HANDLES*************************************/

typedef struct {
  adxl345_power_mode power_mode;
  adxl345_resolution resolution;
  adxl345_scale scale;
  adxl345_odr odr;
  bool is_Setup;
} adxl345_dev;

typedef struct {
  adxl345_power_mode power_mode;
  adxl345_resolution resolution;
  adxl345_scale scale;
  adxl345_odr odr;
  bool is_Setup;
} adxl345_init_param;

/*******************************PROTOTYPES*************************************/

int8_t adxl345_setup(adxl345_dev *dev, adxl345_init_param adxl345_params);

bool adxl345_online(void);

#endif