#include "stdint.h"

#include "cmock.h"
#include "unity.h"

#include "adxl345.h"
#include "mock_i2c.h"

static adxl345_dev dev;
static adxl345_init_param init_param;

void test_adxl345_setup(void) {
  i2c_init_ExpectAndReturn(true);
  TEST_ASSERT_EQUAL(ADXL345_STATUS_SUCCESS, adxl345_setup(&dev, init_param));
}

void test_adxl345_online(void) {
  uint8_t read_data_result = 0xE5;
  i2c_read_byte_ExpectAndReturn(ADXL345_I2C_ADDRESS, ADXL345_REG_DEV_ID, NULL,
                                ADXL345_STATUS_SUCCESS);
  i2c_read_byte_IgnoreArg_read_data();
  i2c_read_byte_ReturnThruPtr_read_data(&read_data_result);

  TEST_ASSERT_TRUE(adxl345_online());
  TEST_ASSERT_EQUAL(read_data_result, ADXL345_DEV_ID);
}