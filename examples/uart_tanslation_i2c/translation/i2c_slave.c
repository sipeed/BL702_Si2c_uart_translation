/*
 * i2c_slave.c
 *
 *  Created on: Jul 31, 2017
 *      Author: dianjixz
 */
/***********************/


#include <stdint.h>
#include <stdlib.h>
#include "i2c_slave.h"
#include "hal_gpio.h"

#include "hal_uart.h"
#include "buff.h"
#include "misc.h"


#define io1_HIGH ((*(volatile uint32_t *)0x40000188) |= (1 << 1))
#define io1_LOW ((*(volatile uint32_t *)0x40000188) &= (~(1 << 1)))





#define sda_io GPIO_PIN_0
#define scl_io GPIO_PIN_15


#define GPIO_REG1        (*(volatile uint32_t *)0x40000100)     //SDA io控制寄存器  GPIO_0
#define GPIO_REG2        (*(volatile uint32_t *)0x4000011C)     //SCL io控制寄存器  GPIO_15
#define GPIO_REG3        (*(volatile uint32_t *)0x40000190)     //SDA和SCL io控制寄存器




#define I2C_SDA_IN        do{ \
                              uint32_t reg_val = GPIO_REG1;\
                              reg_val &= 0xFFFF0000;\
                              reg_val |= 0x00000B1F ;\
                              GPIO_REG1 = reg_val;\
                              reg_val = GPIO_REG3;\
                              reg_val &= ~ 0x00000001;\
                              GPIO_REG3 = reg_val;\
                                }while(0)

#define I2C_SDA_OUT       do{ \
                              uint32_t reg_val = GPIO_REG1;\
                              reg_val = reg_val & 0xFFFF0000;\
                              reg_val = reg_val | 0x00000B1C ;\
                              GPIO_REG1 = reg_val;\
                              reg_val = GPIO_REG3;\
                              reg_val = reg_val | 0x00000001;\
                              GPIO_REG3 = reg_val;\
                                }while(0)


#define I2C_SCL_IN        do{ \
                              uint32_t reg_val = GPIO_REG2;\
                              reg_val = reg_val & 0x0000FFFF;\
                              reg_val = reg_val | 0x0B1F0000 ;\
                              GPIO_REG2 = reg_val;\
                              reg_val = GPIO_REG3;\
                              reg_val = reg_val & (~ 0x00008000);\
                              GPIO_REG3 = reg_val;\
                                }while(0)


#define I2C_SCL_OUT       do{ \
                              uint32_t reg_val = GPIO_REG2;\
                              reg_val = reg_val & 0x0000FFFF;\
                              reg_val = reg_val | 0x0B1C0000 ;\
                              GPIO_REG2 = reg_val;\
                              reg_val = GPIO_REG3;\
                              reg_val = reg_val | 0x00008000;\
                              GPIO_REG3 = reg_val;\
                                }while(0)




// #define I2C_SCL_IN        gpio_set_mode(scl_io, GPIO_INPUT_MODE)
// #define I2C_SCL_OUT       gpio_set_mode(sda_io, GPIO_OUTPUT_PP_MODE)
// #define I2C_SDA_IN        gpio_set_mode(sda_io, GPIO_INPUT_MODE)
// #define I2C_SDA_OUT       gpio_set_mode(sda_io, GPIO_OUTPUT_PP_MODE)


#define SDA_da            ((*(volatile uint32_t *)0x40000180) & 1)
#define SCL_da            ((*(volatile uint32_t *)0x40000180) & 1 << 15)

#define I2C_SCL_IO        SCL_da >> 15
#define I2C_SDA_IO        SDA_da

#define I2C_SCL_HIGH      ((*(volatile uint32_t *)0x40000188) |= (1 << scl_io))
#define I2C_SCL_LOW       ((*(volatile uint32_t *)0x40000188) &= (~(1 << scl_io)))
#define I2C_SDA_HIGH      ((*(volatile uint32_t *)0x40000188) |= (1 << sda_io))
#define I2C_SDA_LOW       ((*(volatile uint32_t *)0x40000188) &= (~(1 << sda_io)))
#define INTTERUPT_OFF     disable_irq()
#define INTTERUPT_ON      enable_irq()


struct MY_I2C_SLAVE my_i2c_slave;



extern int i2c_flages;

static int i2c_wait_scl(int val);
static uint8_t i2c_read_byte();
static void i2c_sda_set(int32_t val);
static void i2c_write_byte(uint8_t val);
static void i2c_slave_send_ack();
static void i2c_slave_get_ack();
static void i2c_pins_init(void);
static void i2c_slave_interrupt(void);


void my_i2c_slave_init(void)
{
  //gpio_init;
  struct MY_I2C_SLAVE *myslave = &my_i2c_slave;
  myslave->i2c_flage = 0;
  myslave->data_offs = 0;
  i2c_pins_init();
  my_i2c_slave.i2c_flage = 0x81;          //#程序加速处理
  i2c_slave_interrupt();
  my_i2c_slave.i2c_flage = 0xE0;
  i2c_slave_interrupt();
  myslave->data_offs = 0;
  while (I2C_SCL_IO == IO_LOW || I2C_SDA_IO == IO_LOW);
}

void i2c_event_selet(void)
{
  INTTERUPT_OFF;
  if(I2C_SCL_IO == IO_HIGH && I2C_SDA_IO == IO_LOW)
  {
    my_i2c_slave.i2c_flage |= I2C_STATE_USE;
    i2c_slave_interrupt();
  }
  INTTERUPT_ON;
}

static void i2c_slave_interrupt(void)
{
  struct MY_I2C_SLAVE *myslave = &my_i2c_slave;
  uint8_t data = 0;
  for (;;)
  {
    switch (myslave->i2c_flage)
    {
    case 0x80:    //i2c总线开始活动,读取i2c总线上的地址
      i2c_wait_scl(IO_LOW);
      data = i2c_read_byte();
      if((data>>1) == I2C_SLAVE_ADDRESS)          //判断地址是否符合
      {
        i2c_slave_send_ack();                     //地址符合后发送应答
        myslave->i2c_flage |= I2C_STATE_DEVICE;   //置总线设备标志位
        if(data & 0x01)                           //判断总线读写,并置读写位
        {
            myslave->i2c_flage |= I2C_STATE_WR;
        }
      }
      else          //地址如果不符,进入下一阶段
      {
        myslave->i2c_flage ++;
      }
      break;
    case 0x81:    //地址不符,持续读取总线上的数据
      i2c_read_byte();
      break;
    case 0xC0:    //主机写  接受数据
      data = i2c_read_byte();
      myslave->i2c_flage ++;
      break;
    case 0xC1:    //主机写 发送应答并存储数据
      i2c_slave_send_ack();
      myslave->i2c_flage --;
      myslave->data[myslave->data_offs] = data;
      myslave->data_offs ++;
      i2c_flages = 1;
      break;
    case 0xE0:    //主机读
      data = buf_pop();
      i2c_write_byte(data);
      myslave->i2c_flage ++;
      break;
    case 0xE1:    //主机读
      i2c_slave_get_ack();
      myslave->i2c_flage ++;
      break;

    case 0xE2 :   //无应答
      i2c_wait_scl(IO_HIGH);
      uint32_t i = I2C_SLAVE_TIMEOUT;
      while (i --)
      {
        if(I2C_SDA_IO == IO_HIGH)
        {
          goto end;
        }
      }
      goto end;
      break;

    case 0xF2:    //有应答
      data = buf_pop();
      i2c_write_byte(data);
      myslave->i2c_flage ++;
      break;
    case 0xF3:
      i2c_slave_get_ack();
      myslave->i2c_flage --;
      break;

    default:
      if(i2c_wait_scl(IO_HIGH) < 0) goto end;
      uint32_t timesc = I2C_SLAVE_TIMEOUT;
      while (timesc --)
      {
        if(I2C_SDA_IO == IO_HIGH)
        {
          goto end;
        }
      }
      goto end;
      break;
    }
  }
  end:
  i2c_pins_init();
  myslave->i2c_flage = 0;
}


static void i2c_pins_init(void)
{
  I2C_SDA_IN;
  I2C_SCL_IN;
}


static int i2c_wait_scl(int val)
{
  struct MY_I2C_SLAVE *myslave = &my_i2c_slave;
  uint32_t i = I2C_SLAVE_TIMEOUT;
  while (i --)
  {
    if(I2C_SCL_IO == val) return 0;
  }
  myslave->i2c_flage |= I2C_STATE_TIMEOUT;
  return -1;
}


static uint8_t i2c_read_byte()
{
  struct MY_I2C_SLAVE *myslave = &my_i2c_slave;
  uint8_t val;
  int count;
  int32_t temp;

  for (int i = 0; i < 8; i++)
  {
    //TODO:timeout check
    if(i2c_wait_scl(IO_HIGH) < 0) return 0;

    val = (val << 1) | I2C_SDA_IO;

    count = I2C_SLAVE_TIMEOUT;
    while (I2C_SCL_IO)
    { //等待高点平结束，判断是否出现异常情况
      //TODO:timeout check
      /* sda is drivered by master now.
       * if it changes when scl is high,stop or start happened */
      if ((count--) == 0)
      {
        myslave->i2c_flage |= I2C_STATE_TIMEOUT;
        return 0;
      }
      temp = I2C_SDA_IO; //读取数据线
      if (!I2C_SCL_IO) break;   //当时钟线为低电平时退出循环
      if ((val & 0x01) != (temp & 0x01)) //判断数据线是否发生了跳变
      {
        if (temp) //数据线被释放
        {
          myslave->i2c_flage &= ~I2C_STATE_USE;           //低到高跳变,总线被释放
        }
        else
        {
          myslave->i2c_flage = I2C_STATE_USE;     //高到低跳变改变总线状态为读取地址状态
          i2c_wait_scl(IO_LOW);
        }
        return 0;
      }
    }
  }
  return val;
}


static void i2c_sda_set(int32_t val)
{
  if (val)
  {
    I2C_SDA_IN;
  }
  else
  {
    I2C_SDA_LOW;
    I2C_SDA_OUT;
  }
}


static void i2c_write_byte(uint8_t val)
{
  uint8_t data = 0x80;
  for (int i = 0; i < 8; i++)
  {
    i2c_sda_set(val & data);
    data = data >> 1;
    if (i2c_wait_scl(1) < 0) return ;
    if (i2c_wait_scl(0) < 0) return ;
  }
  i2c_sda_set(IO_HIGH);
}


static void i2c_slave_send_ack()
{

  /* slave driver sda to low for ACK */
  i2c_sda_set(IO_LOW);
  /* wait master read(scl rising edge trigger) ACK */
  /* wait scl to HIGH */
  if (i2c_wait_scl(IO_HIGH) < 0) return ;
  /* wait scl to low */
  if (i2c_wait_scl(IO_LOW) < 0) return ;

  i2c_sda_set(1);
}

static void i2c_slave_get_ack()
{
  struct MY_I2C_SLAVE *myslave = &my_i2c_slave;
  if (i2c_wait_scl(IO_HIGH) < 0) return ;
  if(I2C_SDA_IO)
  {
    myslave->i2c_flage &= ~I2C_STATE_ACK;
  }
  else
  {
    myslave->i2c_flage |= I2C_STATE_ACK;
  }
  i2c_wait_scl(IO_LOW);
}







/***********************/