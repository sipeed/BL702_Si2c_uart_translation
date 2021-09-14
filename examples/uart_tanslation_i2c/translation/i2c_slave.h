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
#define I2C_SLAVE_TIMEOUT 3000


#define I2C_STATE_USE 0x80
#define I2C_STATE_DEVICE 0x40
#define I2C_STATE_WR 0x20
#define I2C_STATE_ACK 0x10
#define I2C_STATE_TIMEOUT 0x08



// #define IO_LOW 0x00000000
// #define IO_HIGH 0x00000001

#define IO_LOW 0
#define IO_HIGH 1


#define I2C_SLAVE_ADDRESS 0x14



struct MY_I2C_SLAVE
{
  uint8_t i2c_flage;
  uint32_t data_offs;
  uint8_t data[I2C_BUFFER_MAX];
};



void my_i2c_slave_init(void);
void i2c_event_selet(void);






// int32_t i2c_slave_init(void);
// int32_t i2c_slave_sda_interrupt_callback() __attribute__((optimize(gcc_good)));
// uint32_t i2c_send_data(uint8_t send_data) __attribute__((optimize(gcc_good)));

extern struct MY_I2C_SLAVE my_i2c_slave;




#endif /* I2C_SLAVE_H_ */
