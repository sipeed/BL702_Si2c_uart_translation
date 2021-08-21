/*
 * i2c_slave.h
 *
 *  Created on: Jul 31, 2017
 *      Author: epanda
 */

#ifndef I2C_SLAVE_H_
#define I2C_SLAVE_H_

#define gcc_good "O3"

#define I2C_BUFFER_MAX (4 * 1024) //缓冲区大小

struct i2c_device
{
  uint32_t addr;
  uint32_t data_offs;
  uint8_t data[I2C_BUFFER_MAX];
  // uint8_t send_data[256];

  /* something such as data field
   * changed flag, write protect.ect.. */
};

struct i2c_slave
{
  int32_t state;
  struct i2c_device dev;
};



int32_t i2c_slave_init(void);
int32_t i2c_slave_sda_interrupt_callback() __attribute__((optimize(gcc_good)));
// uint32_t i2c_send_data(uint8_t send_data) __attribute__((optimize(gcc_good)));
extern struct i2c_slave my_slave;




#endif /* I2C_SLAVE_H_ */
