
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

#ifdef PLATFORM_ZEPHYR
#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/kernel.h>

#define i2c0_master DT_NODELABEL(i2c0)
#endif

typedef enum {
  ADXL345_STATUS_SUCCESS   = 0,
  ADXL345_STATUS_API_ERR   = -1,
  ADXL345_STATUS_INPUT_ERR = -2,
  ADXL345_STATUS_ALLOC_ERR = -3,
} ADXL345_RETURN_STATUS;

typedef struct {
#ifdef PLATFORM_ZEPHYR
  struct device *i2c0_dev;
#endif
  bool is_Setup;
} adxl345_dev;

typedef struct {
#ifdef PLATFORM_ZEPHYR
  struct device *i2c0_dev;
#endif
  bool is_Setup;
} adxl345_init_param;

int8_t adxl345_setup(adxl345_dev *dev, adxl345_init_param adxl345_params);

int8_t adxl345_i2c_read(adxl345_dev *device, uint8_t address, uint8_t reg,
                        uint8_t *read_data);
int8_t adxl345_i2c_write(adxl345_dev *device, uint8_t address,
                         uint8_t *data_buffer);
#endif