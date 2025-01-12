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

#ifdef PLATFORM_ZEPHYR
  dev->i2c0_dev = (struct device *)DEVICE_DT_GET(i2c0_master);

  if (!device_is_ready(dev->i2c0_dev)) {
    return ADXL345_STATUS_API_ERR;
  }
#endif

  return 0;
}

/**
 * @brief Reads a register value from the ADXL345 device via I2C.
 *
 * Reads a single register from the ADXL345 device and stores the retrieved
 * value in the specified buffer.
 *
 * @param device      Pointer to the ADXL345 device structure.
 * @param address     I2C address of the ADXL345 device.
 * @param reg         Register address to read from.
 * @param read_data   Pointer to the buffer where the read value will be stored.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_i2c_read(adxl345_dev *device, uint8_t address, uint8_t reg,
                        uint8_t *read_data) {
#ifdef PLATFORM_ZEPHYR
  uint32_t bytecount = 1;

  if (i2c_write(device->i2c0_dev, &reg, bytecount, address) !=
      LSM303_STATUS_SUCCESS) {
    return LSM303_STATUS_API_ERR;
  }

  if (i2c_read(device->i2c0_dev, read_data, sizeof(*read_data), address) != 0) {
    return LSM303_STATUS_API_ERR;
  }

  return LSM303_STATUS_SUCCESS;
#endif
}

/**
 * @brief Writes data to a register of the ADXL345 device over I2C.
 *
 * This function writes data to a specified register of the LSM303 device
 * using the provided data buffer.
 *
 * @param device      Pointer to the ADXL345 device structure.
 * @param address     I2C address of the ADXL345 device.
 * @param data_buffer Pointer to the buffer containing the data to be written.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_i2c_write(adxl345_dev *device, uint8_t address,
                         uint8_t *data_buffer) {
#ifdef PLATFORM_ZEPHYR
  uint32_t bytecount = 2;
  return i2c_write(device->i2c0_dev, data_buffer, bytecount, address);
#endif
}
