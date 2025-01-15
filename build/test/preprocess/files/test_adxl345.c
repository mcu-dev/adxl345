#include "build/test/mocks/mock_i2c.h"
#include "src/adxl345.h"
#include "C:/Ruby30-x64/lib/ruby/gems/3.0.0/gems/ceedling-0.31.1/vendor/unity/src/unity.h"
#include "C:/Ruby30-x64/lib/ruby/gems/3.0.0/gems/ceedling-0.31.1/vendor/cmock/src/cmock.h"








static adxl345_dev dev;

static adxl345_init_param init_param;



void test_adxl345_setup(void) {

  i2c_init_CMockExpectAndReturn(13, 

 1

 );

  UnityAssertEqualNumber((UNITY_INT)((ADXL345_STATUS_SUCCESS)), (UNITY_INT)((adxl345_setup(&dev, init_param))), (

 ((void *)0)

 ), (UNITY_UINT)(14), UNITY_DISPLAY_STYLE_INT);

}



void test_adxl345_online(void) {

  uint8_t read_data_result = 0x0E;

  i2c_read_byte_CMockExpectAndReturn(

                                                      20

  , 0x53, 0x00, 

 ((void *)0)

 , ADXL345_STATUS_SUCCESS)

                                                       ;

  i2c_read_byte_CMockIgnoreArg_read_data(21);

  i2c_read_byte_CMockReturnMemThruPtr_read_data(22, &read_data_result, sizeof(uint8_t));



  do {if ((adxl345_online())) {} else {UnityFail( ((" Expected TRUE Was FALSE")), (UNITY_UINT)((UNITY_UINT)(24)));}} while(0);

  UnityAssertEqualNumber((UNITY_INT)((read_data_result)), (UNITY_INT)((0X0E)), (

 ((void *)0)

 ), (UNITY_UINT)(25), UNITY_DISPLAY_STYLE_INT);

}
