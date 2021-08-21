/*
 * i2c_slave.c
 *
 *  Created on: Jul 31, 2017
 *      Author: epanda
 */

#include <stdint.h>
#include <stdlib.h>
#include "i2c_slave.h"
#include "hal_gpio.h"

#include "hal_uart.h"
#include "buff.h"
#define sda_io GPIO_PIN_0
#define scl_io GPIO_PIN_15

#define io1_HIGH ((*(volatile uint32_t *)0x40000188) |= (1 << 1))
#define io1_LOW ((*(volatile uint32_t *)0x40000188) &= (~(1 << 1)))

#define SDA_HIGH ((*(volatile uint32_t *)0x40000188) |= (1 << sda_io))
#define SDA_LOW ((*(volatile uint32_t *)0x40000188) &= (~(1 << sda_io)))
#define SCL_HIGH ((*(volatile uint32_t *)0x40000188) |= (1 << scl_io))
#define SCL_LOW ((*(volatile uint32_t *)0x40000188) &= (~(1 << scl_io)))

#define SDA_da ((*(volatile uint32_t *)0x40000180) & 1)
#define SCL_da ((*(volatile uint32_t *)0x40000180) & 1 << 15)

#define SDA_INPUT SDA_da

#define SCL_INPUT SCL_da >> 15

#define SDA_OUT gpio_set_mode(sda_io, GPIO_OUTPUT_PP_MODE)
#define SDA_IN gpio_set_mode(sda_io, GPIO_INPUT_MODE)
#define SCL_OUT gpio_set_mode(scl_io, GPIO_OUTPUT_PP_MODE)
#define SCL_IN gpio_set_mode(scl_io, GPIO_INPUT_MODE)

#define i2c_slave_addr 0x14

#define I2C_STATE_NACK 1
#define I2C_STATE_ACK 0
#define I2C_STATE_IDLE -1
#define I2C_STATE_START -2
#define I2C_STATE_STOP -3
#define I2C_STATE_DEVICE -4
#define I2C_STATE_TIMEOUT -5

#define I2C_RET_OK 0
#define I2C_RET_END -1

#define I2C_DEV_OFFS 0x0
#define I2C_DEV_DATA 0x1

static inline int32_t slave_data_receive(struct i2c_slave *slave) __attribute__((optimize(gcc_good)));
static inline int32_t slave_data_send(struct i2c_slave *slave) __attribute__((optimize(gcc_good)));
static inline int slave_byte_read(struct i2c_slave *slave, uint8_t *data) __attribute__((optimize(gcc_good)));
static inline int32_t slave_byte_write(struct i2c_slave *slave, uint8_t val) __attribute__((optimize(gcc_good)));
static inline int32_t i2c_dev_address_check(struct i2c_slave *slave, int32_t val) __attribute__((optimize(gcc_good)));
static inline int i2c_ack_send(struct i2c_slave *slave) __attribute__((optimize(gcc_good)));
static inline int32_t i2c_ack_read(struct i2c_slave *slave) __attribute__((optimize(gcc_good)));
static inline void i2c_pins_init(void) __attribute__((optimize(gcc_good)));
static inline int32_t i2c_scl_get() __attribute__((optimize(gcc_good)));
static inline int32_t i2c_sda_get() __attribute__((optimize(gcc_good)));
static inline void i2c_sda_set(int32_t val) __attribute__((optimize(gcc_good)));
static inline int32_t wait_for_scl(struct i2c_slave *slave, int32_t level) __attribute__((optimize(gcc_good)));

extern int i2c_flages;

struct i2c_slave my_slave;

int32_t i2c_slave_init(void)
{
  // uint32_t i, j;
  struct i2c_slave *slave;

  slave = &my_slave;

  slave->dev.addr = i2c_slave_addr;

  slave->dev.data_offs = 0;

  // for (i = 0; i < sizeof(slave->dev.data); i++)
  // {
  //   slave->dev.data[i] = 0;
  // }
  i2c_pins_init();
  return 0;
}
int32_t i2c_slave_sda_interrupt_callback()
{
  volatile int32_t val;
  uint8_t byte;
  struct i2c_slave *slave;
  volatile int32_t idx;
  volatile int count;
  slave = &my_slave;

  //wait scl HIGH
  if (wait_for_scl(slave, 0) == I2C_RET_END)
  {
    /* timeout */
    goto end;
  }
  slave->state = I2C_STATE_START;
  while (slave->state == I2C_STATE_START)
  {
    /* read address + R/W bit */
    if (slave_byte_read(slave, &byte) != I2C_RET_OK)
    {           //发生读错误
      goto end; //退出
    }

    idx = i2c_dev_address_check(slave, byte); //检查地址

    if (idx == I2C_RET_END) //地址错误
    {
      /* device address mismatch */
      goto end;
    }

    slave->state = I2C_STATE_DEVICE;

    /* send ACK */
    if (i2c_ack_send(slave) == I2C_RET_END)
    {
      goto end; //应答错误
    }
    /* check R/W bit */
    val = byte;
    if (val & 1)
    {
      //TODO: bug.master can read data without pre-send device data offset
      val = slave_data_send(slave);
      if (slave->state == I2C_STATE_START)
      {
        continue;
      }
      else
      {
        goto end;
      }
    }
    else
    {
      val = slave_data_receive(slave);
      if (slave->state == I2C_STATE_START)
      {
        continue;
      }
      else
      {
        i2c_flages = 1;
        goto end;
      }
    }
  }
end:
  count = 5000;
  /* wait scl and sda high */
  while (!(i2c_scl_get() && i2c_sda_get()))
  {
    //TODO:timeout check
    if ((count--) == 0)
    {
      break;
    }
  }
  i2c_pins_init();
  slave->state = I2C_STATE_IDLE;
  return 0;
}

static inline int32_t i2c_scl_get()
{
  return SCL_INPUT;
}

static inline void i2c_sda_set(int32_t val)
{
  if (val)
    SDA_IN;
  else
  {
    SDA_OUT;
    SDA_LOW;
  }
}

static inline int32_t i2c_sda_get(void)
{
  return SDA_INPUT;
}

static inline void i2c_pins_init(void)
{
  SDA_IN;
  SCL_IN;
}

static void i2c_slave_store_data(struct i2c_slave *slave, uint8_t flag, uint8_t data)
{
  switch (flag)
  {
  case I2C_DEV_OFFS:
    // slave->dev.data_offs = data;
    slave->dev.data_offs = 0;
    slave->dev.data[slave->dev.data_offs] = data;
    slave->dev.data_offs++;
    break;
  case I2C_DEV_DATA:
    slave->dev.data[slave->dev.data_offs] = data;
    slave->dev.data_offs++;
    break;

  default:
    break;
  }
}

static inline int32_t i2c_dev_address_check(struct i2c_slave *slave, int32_t val)
{
  if (slave->dev.addr == (val >> 1))
    return I2C_RET_OK;
  return I2C_RET_END;
}

static inline int32_t wait_for_scl(struct i2c_slave *slave, int32_t level)
{
  volatile uint32_t i = 3000;
  while (i2c_scl_get() != level)
  {
    if (--i == 0)
    {
      slave->state = I2C_STATE_TIMEOUT;
      return I2C_RET_END;
    }
  }
  return I2C_RET_OK;
}

static inline int32_t slave_byte_write(struct i2c_slave *slave, uint8_t val)
{
  volatile uint8_t i;
  volatile uint8_t data = 0x80;
  for (i = 0; i < 8; i++)
  {
    i2c_sda_set(val & data);
    data = data >> 1;
    if (wait_for_scl(slave, 1) == I2C_RET_END)
    {
      i2c_sda_set(1);
      return I2C_RET_END;
    }
    if (wait_for_scl(slave, 0) == I2C_RET_END)
    {
      i2c_sda_set(1);
      return I2C_RET_END;
    }
  }
  i2c_sda_set(1);
  return I2C_RET_OK;
}

static inline int slave_byte_read(struct i2c_slave *slave, uint8_t *data)
{
  uint8_t i;
  volatile int32_t val, temp;
  volatile int count;
  for (i = 0x0; i < 0x8; i++)
  {
    //TODO:timeout check
    if (wait_for_scl(slave, 1) != I2C_RET_OK)
    {
      return I2C_RET_END;
    }
    val = (val << 0x1) | i2c_sda_get();
    count = 3000;
    while (i2c_scl_get())
    { //等待高点平结束，判断是否出现异常情况
      //TODO:timeout check
      /* sda is drivered by master now.
       * if it changes when scl is high,stop or start happened */
      if ((count--) == 0)
      {
        slave->state = I2C_STATE_TIMEOUT;
        return I2C_RET_END;
      }
      temp = i2c_sda_get(); //读取数据线
      if (!i2c_scl_get())   //当时钟线为低电平时退出循环
        break;
      if ((val & 1) != temp) //判断数据线是否发生了跳变
      {
        if (temp) //数据线被释放
        {
          slave->state = I2C_STATE_STOP;
        }
        else
        {
          slave->state = I2C_STATE_START;
          wait_for_scl(slave, 0);
        }
        return I2C_RET_END;
      }
    }
  }
  *data = val;
  return I2C_RET_OK;
}
static inline int i2c_ack_send(struct i2c_slave *slave)
{
  /* slave driver sda to low for ACK */
  i2c_sda_set(0);
  /* wait master read(scl rising edge trigger) ACK */
  /* wait scl to HIGH */
  if (wait_for_scl(slave, 1) != I2C_RET_OK)
  {
    i2c_sda_set(1);
    return I2C_RET_END;
  }
  /* wait scl to low */
  if (wait_for_scl(slave, 0) != I2C_RET_OK)
  {
    i2c_sda_set(1);
    return I2C_RET_END;
  }
  i2c_sda_set(1);
  return I2C_RET_OK;
}

static inline int32_t i2c_ack_read(struct i2c_slave *slave)
{
  volatile int32_t val, temp;
  volatile int count;
  /* wait master set sda */
  if (wait_for_scl(slave, 1) == I2C_RET_END)
  {
    slave->state = I2C_STATE_STOP;
    return I2C_RET_END;
  }
  count = 2000;
  val = i2c_sda_get();
  while (i2c_scl_get())
  {
    if ((count--) == 0)
    {
      slave->state = I2C_STATE_TIMEOUT;
      return I2C_RET_END;
    }
    temp = i2c_sda_get();
    if (!i2c_scl_get())
      break;

    if (val != temp)
    {
      if (temp)
      {
        slave->state = I2C_STATE_STOP;
      }
      else
      {
        slave->state = I2C_STATE_START;
      }
      return I2C_RET_END;
    }
  }
  if (val == 0x0)
  {
    /* ACK */
    slave->state = I2C_STATE_ACK;
    return I2C_RET_OK;
  }
  else
  { //如果没有应答，接收停止位
    /* NACK */
    slave->state = I2C_STATE_NACK;
    /* wait master set sda */
    if (wait_for_scl(slave, 1) == I2C_RET_END)
    {
      return I2C_RET_END;
    }
    count = 5000;
    val = i2c_sda_get();
    while (i2c_scl_get())
    {
      if ((count--) == 0)
      {
        slave->state = I2C_STATE_TIMEOUT;
        return I2C_RET_END;
      }
      temp = i2c_sda_get();
      if (!i2c_scl_get())
        break;

      if (val != temp)
      {
        if (temp)
        {
          slave->state = I2C_STATE_STOP;
        }
        else
        {
          slave->state = I2C_STATE_START;
        }
        return I2C_RET_END;
      }
    }
    return I2C_RET_END;
  }
}
static inline int32_t slave_data_send(struct i2c_slave *slave)
{
  uint8_t val;
  do
  {
    val = buf_pop();
    if (slave_byte_write(slave, val) == I2C_RET_END)
    {
      return I2C_RET_END;
    }
  } while (i2c_ack_read(slave) == I2C_RET_OK);
  return I2C_RET_OK;
}

static inline int32_t slave_data_receive(struct i2c_slave *slave)
{
  // volatile int32_t val = 0;
  uint8_t byte;
  uint8_t flag = I2C_DEV_OFFS;
  while (slave_byte_read(slave, &byte) != I2C_RET_END)
  {
    if (i2c_ack_send(slave) == I2C_RET_END)
    {
      return I2C_RET_END;
    }

    i2c_slave_store_data(slave, flag, byte);
    flag = I2C_DEV_DATA;
  }

  return I2C_RET_OK;
}
