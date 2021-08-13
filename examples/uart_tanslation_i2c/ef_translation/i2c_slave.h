/*
 * i2c_slave.h
 *
 *  Created on: Jul 31, 2017
 *      Author: epanda
 */

#ifndef I2C_SLAVE_H_
#define I2C_SLAVE_H_

#define gcc_good "O3"

struct i2c_device
{
  uint8_t addr;
  uint8_t data_offs;
  uint8_t data[256];
  uint8_t send_data[256];

  /* something such as data field
   * changed flag, write protect.ect.. */
};

struct I2c_Data
{
  uint8_t A_Data[256];
  uint8_t B_Data[256];
  
  uint8_t A_FIFO;
  uint8_t B_FIFO;
  
  uint8_t A_head_count;
  uint8_t A_end_count;
  uint8_t B_head_count;
  uint8_t B_end_count;

  uint8_t AorB_Status;
  uint8_t STC_A;
  uint8_t STC_B;
  
};

struct i2c_slave
{
  int32_t state;
  struct i2c_device dev;
  struct I2c_Data Data;
};



int32_t i2c_slave_init(void);
int32_t i2c_slave_sda_interrupt_callback() __attribute__((optimize(gcc_good)));
uint32_t i2c_send_data(uint8_t send_data , uint8_t send_count , uint8_t send_statu) __attribute__((optimize(gcc_good)));
uint32_t i2c_send_status_Flip(void) __attribute__((optimize(gcc_good)));
uint32_t STC_GET(void) __attribute__((optimize(gcc_good)));
extern struct i2c_slave my_slave;




#endif /* I2C_SLAVE_H_ */
