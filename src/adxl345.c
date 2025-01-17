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
 * @brief Write a register value from the ADXL345 accelerometer.
 *
 * @param dev_addr The register address to write the value from.
 * @param data_buffer Pointer to the buffer containing the data to be written.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */

int8_t adxl345_write_register_value(uint8_t dev_addr, uint8_t *data_buffer) {
  return i2c_write_bytes(dev_addr, data_buffer);
}

/**
 * @brief Read a register value from the ADXL345 accelerometer.
 *
 * @param address The register address to read the value from.
 * @param val Pointer to a variable to store the read register value.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_read_register_value(uint8_t address, uint8_t *val) {
  if (i2c_read_byte(ADXL345_I2C_ADDRESS, address, val) !=
      ADXL345_STATUS_SUCCESS) {
    return ADXL345_STATUS_API_ERR;
  }
  return ADXL345_STATUS_SUCCESS;
}

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

/**
 * @brief Sets the accelerometer power mode.
 *
 * Configures the accelerometer's power mode to optimize power consumption
 * or performance based on the specified mode.
 *
 * @param device Pointer to the ADXL345 device structure.
 * @param mode   Power mode to be configured for the accelerometer.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_set_power_mode(adxl345_dev *device, adxl345_power_mode mode) {
  uint8_t val = 0x00;

  if (i2c_read_byte(ADXL345_I2C_ADDRESS, ADXL345_REG_BW_RATE, &val) !=
      ADXL345_STATUS_SUCCESS) {
    return ADXL345_STATUS_API_ERR;
  }

  val &= ~0x10;
  val = val | mode << ADXL345_POWER_MODE_MASK;

  device->power_mode = mode;

  uint8_t data_buffer[] = {ADXL345_REG_BW_RATE, val};
  return i2c_write_bytes(ADXL345_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Sets the accelerometer measurement mode.
 *
 * Configures the accelerometer's measurement mode
 *
 * @param device Pointer to the ADXL345 device structure.
 * @param mode   Power mode to be configured for the accelerometer.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_set_measure_mode(adxl345_dev *device,
                                adxl345_measure_mode measure) {
  uint8_t val = 0x00;

  if (i2c_read_byte(ADXL345_I2C_ADDRESS, ADXL345_REG_POWER_CTL, &val) !=
      ADXL345_STATUS_SUCCESS) {
    return ADXL345_STATUS_API_ERR;
  }

  val &= ~0x08;
  val = val | measure << ADXL345_MEASURE_MASK;

  device->measure = measure;

  uint8_t data_buffer[] = {ADXL345_REG_POWER_CTL, val};
  return i2c_write_bytes(ADXL345_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Configures the Output Data Rate (ODR) for the accelerometer.
 *
 * Sets the output data rate of the accelerometer to control how frequently
 * the device outputs data.
 *
 * @param device Pointer to the ADXL345 device structure.
 * @param mode   Power mode to be configured for the accelerometer.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_set_odr(adxl345_dev *device, adxl345_odr odr) {
  uint8_t val = 0x00;

  if (device->power_mode == ADXL345_LOW_POWER_MODE) {
    if (odr > ADXL345_ODR_400HZ) {
      // Refer to Table 8
      return ADXL345_STATUS_INPUT_ERR;
    }
  }

  if (i2c_read_byte(ADXL345_I2C_ADDRESS, ADXL345_REG_BW_RATE, &val) !=
      ADXL345_STATUS_SUCCESS) {
    return ADXL345_STATUS_API_ERR;
  }

  val &= ~0x0F;
  val = val | odr << ADXL345_ODR_MASK;

  device->odr = odr;

  uint8_t data_buffer[] = {ADXL345_REG_BW_RATE, val};
  return i2c_write_bytes(ADXL345_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Configures the accelerometer scale for the ADXL345 device.
 *
 * Sets the full-scale range of the accelerometer based on the
 * specified scale parameter.
 *
 * @param device Pointer to the ADXL345 device structure.
 * @param mode   Power mode to be configured for the accelerometer.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_set_scale(adxl345_dev *device, adxl345_scale scale) {
  uint8_t val = 0x00;

  if (i2c_read_byte(ADXL345_I2C_ADDRESS, ADXL345_REG_DATA_FORMAT, &val) !=
      ADXL345_STATUS_SUCCESS) {
    return ADXL345_STATUS_API_ERR;
  }

  val &= ~0x03;
  val = val | scale << ADXL345_SCALE_MASK;

  device->scale = scale;

  uint8_t data_buffer[] = {ADXL345_REG_DATA_FORMAT, val};
  return i2c_write_bytes(ADXL345_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Configures the accelerometer resolution for the ADXL345 device.
 *
 * Sets the resolution of the accelerometer based on the specified
 * resolution parameter.
 *
 * @param device Pointer to the LSM303 device structure.
 * @param resolution  Resolution setting for the accelerometer.
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_acc_set_resolution(adxl345_dev *device,
                                  adxl345_resolution resolution) {
  uint8_t val = 0x00;

  if (i2c_read_byte(ADXL345_I2C_ADDRESS, ADXL345_REG_DATA_FORMAT, &val) !=
      ADXL345_STATUS_SUCCESS) {
    return ADXL345_STATUS_API_ERR;
  }

  val &= ~0x08;
  val = val | resolution << ADXL345_RESOLUTION_MASK;

  device->resolution = resolution;

  uint8_t data_buffer[] = {ADXL345_REG_DATA_FORMAT, val};
  return i2c_write_bytes(ADXL345_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Set the tap x-axis offset for the ADXL345 accelerometer. Offset
 * scale factor is 15.6mg/LSB
 *
 * @param device Pointer to the ADXL345 device structure.
 * @param offset Offset to be added in the x-axis (-128 to 127)
 *      -128: -128 x 0.0156 g/LSB = -2g
 *         0:    0 x 0.0156 g/LSB =  0g
 *       127:  127 x 0.0156 g/LSB =  2g
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_set_offset_x(adxl345_dev *device, int8_t offset) {
  device->offset_config.x = offset;

  uint8_t data_buffer[] = {ADXL345_REG_OFFSET_X, offset};
  return i2c_write_bytes(ADXL345_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Set the tap y-axis offset for the ADXL345 accelerometer. Offset
 * scale factor is 15.6mg/LSB
 *
 * @param device Pointer to the ADXL345 device structure.
 * @param offset Offset to be added in the y-axis (-128 to 127)
 *      -128: -128 x 0.0156 g/LSB = -2g
 *         0:    0 x 0.0156 g/LSB =  0g
 *       127:  127 x 0.0156 g/LSB =  2g
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_set_offset_y(adxl345_dev *device, int8_t offset) {
  device->offset_config.y = offset;

  uint8_t data_buffer[] = {ADXL345_REG_OFFSET_Y, offset};
  return i2c_write_bytes(ADXL345_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Set the tap z-axis offset for the ADXL345 accelerometer. Offset
 * scale factor is 15.6mg/LSB
 *
 * @param device Pointer to the ADXL345 device structure.
 * @param offset Offset to be added in the z-axis (-128 to 127)
 *      -128: -128 x 0.0156 g/LSB = -2g
 *         0:    0 x 0.0156 g/LSB =  0g
 *       127:  127 x 0.0156 g/LSB =  2g
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_set_offset_z(adxl345_dev *device, int8_t offset) {
  device->offset_config.z = offset;

  uint8_t data_buffer[] = {ADXL345_REG_OFFSET_Z, offset};
  return i2c_write_bytes(ADXL345_I2C_ADDRESS, data_buffer);
}

/**
 * @brief Set the tap threshold for the ADXL345 accelerometer.
 *
 *  Sets the tap threshold for both single and double tap. Tap scale factor
 *  is 62.5mg/LSB
 *
 * @param device Pointer to the ADXL345 device structure.
 * @param threshold The desired tap threshold (0-255)
 *
 * @return
 *   - 0 on success.
 *   - Non-zero error code on failure.
 */
int8_t adxl345_set_tap_threshold(adxl345_dev *device, uint8_t threshold) {
  device->tap_config.threshold = threshold;

  uint8_t data_buffer[] = {ADXL345_REG_THRESH_TAP, threshold};
  return i2c_write_bytes(ADXL345_I2C_ADDRESS, data_buffer);
}

// Add doxygen
int8_t adxl345_set_tap_duration(adxl345_dev *device, int8_t duration) {
  device->tap_config.duration = duration;

  uint8_t data_buffer[] = {ADXL345_REG_DUR, duration};
  return i2c_write_bytes(ADXL345_I2C_ADDRESS, data_buffer);
}

// ADD DOXYGEN
int8_t adxl345_set_tap_latency(adxl345_dev *device, int8_t latency) {
  device->tap_config.latency = latency;

  uint8_t data_buffer[] = {ADXL345_REG_LATENT, latency};
  return i2c_write_bytes(ADXL345_I2C_ADDRESS, data_buffer);
}

// ADD DOXYGEN
int8_t adxl345_set_tap_window(adxl345_dev *device, int8_t window) {
  device->tap_config.window = window;

  uint8_t data_buffer[] = {ADXL345_REG_WINDOW, window};
  return i2c_write_bytes(ADXL345_I2C_ADDRESS, data_buffer);
}

// ADD DOXYGEN
int8_t adxl345_set_activity_threshold(adxl345_dev *device, int8_t threshold) {
  device->activity_config.activity_threshold = threshold;

  uint8_t data_buffer[] = {ADXL345_REG_THRESH_ACT, threshold};
  return i2c_write_bytes(ADXL345_I2C_ADDRESS, data_buffer);
}

// ADD DOXYGEN
int8_t adxl345_set_inactivity_threshold(adxl345_dev *device, int8_t threshold) {
  device->activity_config.inactivity_threshold = threshold;

  uint8_t data_buffer[] = {ADXL345_REG_THRESH_INACT, threshold};
  return i2c_write_bytes(ADXL345_I2C_ADDRESS, data_buffer);
}

// ADD DOXYGEN
int8_t adxl345_enable_axes_activity(adxl345_dev *device,
                                    adxl345_axes_enable activity_en) {
  uint8_t val = 0x00;

  if (i2c_read_byte(ADXL345_I2C_ADDRESS, ADXL345_REG_ACT_INACT_CTL, &val) !=
      ADXL345_STATUS_SUCCESS) {
    return ADXL345_STATUS_API_ERR;
  }

  val &= ~0x70;
  val = val | activity_en << ADXL345_ACTIVITY_EN_MASK;

  device->activity_config.activity_en = activity_en;

  uint8_t data_buffer[] = {ADXL345_REG_ACT_INACT_CTL, val};
  return i2c_write_bytes(ADXL345_I2C_ADDRESS, data_buffer);
}

// ADD DOXYGEN
int8_t adxl345_enable_axes_inactivity(adxl345_dev *device,
                                      adxl345_axes_enable inactivity_en) {
  uint8_t val = 0x00;

  if (i2c_read_byte(ADXL345_I2C_ADDRESS, ADXL345_REG_ACT_INACT_CTL, &val) !=
      ADXL345_STATUS_SUCCESS) {
    return ADXL345_STATUS_API_ERR;
  }

  val &= ~0x07;
  val = val | inactivity_en << ADXL345_INACTIVITY_EN_MASK;

  device->activity_config.inactivity_en = inactivity_en;

  uint8_t data_buffer[] = {ADXL345_REG_ACT_INACT_CTL, val};
  return i2c_write_bytes(ADXL345_I2C_ADDRESS, data_buffer);
}

// ADD DOXYGEN
int8_t adxl345_set_freefall_threshold(adxl345_dev *device, int8_t threshold) {
  device->freefall_config.threshold = threshold;

  uint8_t data_buffer[] = {ADXL345_REG_THRESH_FF, threshold};
  return i2c_write_bytes(ADXL345_I2C_ADDRESS, data_buffer);
}

// ADD DOXYGEN
int8_t adxl345_set_freefall_timeout(adxl345_dev *device, int8_t timeout) {
  device->freefall_config.timeout = timeout;

  uint8_t data_buffer[] = {ADXL345_REG_TIME_FF, timeout};
  return i2c_write_bytes(ADXL345_I2C_ADDRESS, data_buffer);
}

// ADD DOXYGEN
int8_t adxl345_tap_axes_enable(adxl345_dev *device,
                               adxl345_axes_enable tap_en) {
  uint8_t val = 0x00;

  if (i2c_read_byte(ADXL345_I2C_ADDRESS, ADXL345_REG_TAP_AXES, &val) !=
      ADXL345_STATUS_SUCCESS) {
    return ADXL345_STATUS_API_ERR;
  }

  val &= ~0x07;
  val = val | tap_en << ADXL345_TAP_EN_MASK;

  device->tap_config.tap_en = tap_en;

  uint8_t data_buffer[] = {ADXL345_REG_TAP_AXES, val};
  return i2c_write_bytes(ADXL345_I2C_ADDRESS, data_buffer);
}

int8_t adxl345_get_trigger_source(adxl345_dev *device) {
  uint8_t val = 0x00;
  if (i2c_read_byte(ADXL345_I2C_ADDRESS, ADXL345_REG_ACT_TAP_STATUS, &val) !=
      ADXL345_STATUS_SUCCESS) {
    return ADXL345_STATUS_API_ERR;
  }
  return ADXL345_STATUS_SUCCESS;
}