/**
 ******************************************************************************
 * @file    adxl345.c
 * @author  - Anthony E.Raterta
 * @version V1.0.0
 * @date    12-Jan-2025
 * @brief   Contains all the functionalities to control the ADXL345
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

#include "adxl345.h"

/**
 * @brief Initializes and sets up the ADXL345 device.
 *
 * Allocates and initializes the device structure and configures it with the
 * specified initialization parameters.
 *
 * @param device        Double pointer to the ADXL345 device structure to be
 * allocated and initialized.
 * @param adxl345_params Initialization parameters for the ADXL345 device.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_setup(adxl345_dev *dev, adxl345_init_param adxl345_params) {
  int8_t ret = 0;

  if (!i2c_init()) {
    return ADXL345_STATUS_INIT_ERR;
  }

  return ret;
}

/**
 * @brief Checks if the ADXL345 is online
 *
 * Checks if the ADXL345 is online by checking the read-only register DEV_ID
 *
 * @param None.
 *
 * @return `true` connected if the return value is 0x0E,
 *         `false` otherwise (future implementation).
 */
bool adxl345_online(void) {
  uint8_t val = 0x00;

  if (i2c_read_byte(ADXL345_I2C_ADDRESS, 0x00, &val) !=
      ADXL345_STATUS_SUCCESS) {
    return ADXL345_STATUS_API_ERR;
  }
  if (val != ADXL345_DEV_ID) {
    return false;
  }
  return true;
}
