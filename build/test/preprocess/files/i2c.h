

typedef enum {

  I2C_STATUS_SUCCESS = 0,

  I2C_STATUS_ERR = -1,

} I2C_RETURN_STATUS;





_Bool 

    i2c_init(void);



int8_t i2c_write_bytes(uint8_t dev_addr, uint8_t *data_buffer);



int8_t i2c_read_byte(uint8_t dev_addr, uint8_t data_read_virtual_address,

                     uint8_t *read_data);
