/*
 * i2c_slave.c
 *
 *  Created on: Jul 31, 2017
 *      Author: dianjixz
 */

// #include <stdint.h>
// #include <stdlib.h>
// #include "i2c_slave.h"
// #include "hal_gpio.h"

// #include "hal_uart.h"
// #include "buff.h"
// #define sda_io GPIO_PIN_0
// #define scl_io GPIO_PIN_15

// #define io1_HIGH ((*(volatile uint32_t *)0x40000188) |= (1 << 1))
// #define io1_LOW ((*(volatile uint32_t *)0x40000188) &= (~(1 << 1)))

// #define SDA_HIGH ((*(volatile uint32_t *)0x40000188) |= (1 << sda_io))
// #define SDA_LOW ((*(volatile uint32_t *)0x40000188) &= (~(1 << sda_io)))
// #define SCL_HIGH ((*(volatile uint32_t *)0x40000188) |= (1 << scl_io))
// #define SCL_LOW ((*(volatile uint32_t *)0x40000188) &= (~(1 << scl_io)))

// #define SDA_da ((*(volatile uint32_t *)0x40000180) & 1)
// #define SCL_da ((*(volatile uint32_t *)0x40000180) & 1 << 15)

// #define SDA_INPUT SDA_da

// #define SCL_INPUT SCL_da >> 15

// #define SDA_OUT gpio_set_mode(sda_io, GPIO_OUTPUT_PP_MODE)
// #define SDA_IN gpio_set_mode(sda_io, GPIO_INPUT_MODE)
// #define SCL_OUT gpio_set_mode(scl_io, GPIO_OUTPUT_PP_MODE)
// #define SCL_IN gpio_set_mode(scl_io, GPIO_INPUT_MODE)

// #define i2c_slave_addr 0x14

// #define I2C_STATE_NACK 1
// #define I2C_STATE_ACK 0
// #define I2C_STATE_IDLE -1
// #define I2C_STATE_START -2
// #define I2C_STATE_STOP -3
// #define I2C_STATE_DEVICE -4
// #define I2C_STATE_TIMEOUT -5

// #define I2C_RET_OK 0
// #define I2C_RET_END -1

// #define I2C_DEV_OFFS 0x0
// #define I2C_DEV_DATA 0x1

// static inline int32_t slave_data_receive(struct i2c_slave *slave) __attribute__((optimize(gcc_good)));
// static inline int32_t slave_data_send(struct i2c_slave *slave) __attribute__((optimize(gcc_good)));
// static int slave_byte_read(struct i2c_slave *slave, uint8_t *data) __attribute__((optimize(gcc_good)));
// static int32_t slave_byte_write(struct i2c_slave *slave, uint8_t val) __attribute__((optimize(gcc_good)));
// static inline int32_t i2c_dev_address_check(struct i2c_slave *slave, int32_t val) __attribute__((optimize(gcc_good)));
// static inline int i2c_ack_send(struct i2c_slave *slave) __attribute__((optimize(gcc_good)));
// static inline int32_t i2c_ack_read(struct i2c_slave *slave) __attribute__((optimize(gcc_good)));
// static inline void i2c_pins_init(void) __attribute__((optimize(gcc_good)));
// static inline int32_t i2c_scl_get() __attribute__((optimize(gcc_good)));
// static inline int32_t i2c_sda_get() __attribute__((optimize(gcc_good)));
// static inline void i2c_sda_set(int32_t val) __attribute__((optimize(gcc_good)));
// static inline int32_t wait_for_scl(struct i2c_slave *slave, int32_t level) __attribute__((optimize(gcc_good)));

// extern int i2c_flages;

// struct i2c_slave my_slave;


/***********************/


#include <stdint.h>
#include <stdlib.h>
#include "i2c_slave.h"
#include "hal_gpio.h"

#include "hal_uart.h"
#include "buff.h"
#include "misc.h"

#define sda_io GPIO_PIN_0
#define scl_io GPIO_PIN_15



#define I2C_SCL_IN        gpio_set_mode(scl_io, GPIO_INPUT_MODE)
#define I2C_SCL_OUT       gpio_set_mode(sda_io, GPIO_OUTPUT_PP_MODE)
#define I2C_SDA_IN        gpio_set_mode(sda_io, GPIO_INPUT_MODE)
#define I2C_SDA_OUT       gpio_set_mode(sda_io, GPIO_OUTPUT_PP_MODE)


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
        if(data & 0x01)                           //判断总些读写,并置读写位
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
      i2c_wait_scl(1);
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
      i2c_wait_scl(1);
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
    if(i2c_wait_scl(1) < 0) return 0;

    val = (val << 0x1) | I2C_SDA_IO;

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
          i2c_wait_scl(0);
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
  i2c_sda_set(1);
}


static void i2c_slave_send_ack()
{

  /* slave driver sda to low for ACK */
  i2c_sda_set(0);
  /* wait master read(scl rising edge trigger) ACK */
  /* wait scl to HIGH */
  if (i2c_wait_scl(1) < 0) return ;
  /* wait scl to low */
  if (i2c_wait_scl(0) < 0) return ;

  i2c_sda_set(1);
}

static void i2c_slave_get_ack()
{
  struct MY_I2C_SLAVE *myslave = &my_i2c_slave;
  if (i2c_wait_scl(1) < 0) return ;
  if(I2C_SDA_IO)
  {
    myslave->i2c_flage &= ~I2C_STATE_ACK;
  }
  else
  {
    myslave->i2c_flage |= I2C_STATE_ACK;
  }
  i2c_wait_scl(0);
}







/***********************/






































// int32_t i2c_slave_init(void)
// {
//   struct i2c_slave *slave;

//   slave = &my_slave;

//   slave->dev.addr = i2c_slave_addr;

//   slave->dev.data_offs = 0;


//   SDA_HIGH;
//   io1_HIGH;
//   i2c_pins_init();
//   return 0;
// }
// uint8_t test_fal = 0;
// int32_t i2c_slave_sda_interrupt_callback()
// {
//   volatile int32_t val;
//   uint8_t byte;
//   struct i2c_slave *slave;
//   volatile int count;
//   slave = &my_slave;

//   //wait scl HIGH
//   if (wait_for_scl(slave, 0) == I2C_RET_END)
//   {
//     /* timeout */
//     goto end;
//   }
//   slave->state = I2C_STATE_START;
//   while (slave->state == I2C_STATE_START)
//   {
//     /* read address + R/W bit */
//     if (slave_byte_read(slave, &byte) != I2C_RET_OK)
//     {           //发生读错误
//       goto end; //退出
//     }

//     if(test_fal<5)
//     {
//       test_fal ++;
//       goto end;
//     }

//     if (slave->dev.addr != (byte >> 1))//检查地址
//     {
//         /* device address mismatch */
//         goto end;
//     }
//     slave->state = I2C_STATE_DEVICE;

//     /* send ACK */
//     if (i2c_ack_send(slave) == I2C_RET_END)
//     {
//       goto end; //应答错误
//     }
//     /* check R/W bit */
//     val = byte;
//     if (val & 1)
//     {
//       //TODO: bug.master can read data without pre-send device data offset
//       val = slave_data_send(slave);
//       if (slave->state == I2C_STATE_START)
//       {
//         continue;
//       }
//       else
//       {
//         goto end;
//       }
//     }
//     else
//     {
//       val = slave_data_receive(slave);
//       if (slave->state == I2C_STATE_START)
//       {
//         continue;
//       }
//       else
//       {
//         i2c_flages = 1;
//         goto end;
//       }
//     }
//   }
// end:
//   count = 5000;
//   /* wait scl and sda high */
//   while (!(i2c_scl_get() && i2c_sda_get()))
//   {
//     //TODO:timeout check
//     if ((count--) == 0)
//     {
//       break;
//     }
//   }
//   i2c_pins_init();
//   slave->state = I2C_STATE_IDLE;
//   return 0;
// }

// static inline int32_t i2c_scl_get()
// {
//   return SCL_INPUT;
// }

// static inline void i2c_sda_set(int32_t val)
// {
//   if (val)
//   {
//     io1_HIGH;
//     SDA_IN;
//   }
//   else
//   {
//     io1_LOW;
//     SDA_LOW;
//     SDA_OUT;
//   }
// }

// static inline int32_t i2c_sda_get(void)
// {
//   return SDA_INPUT;
// }

// static inline void i2c_pins_init(void)
// {
//   SDA_IN;
//   SCL_IN;
// }

// static void i2c_slave_store_data(struct i2c_slave *slave, uint8_t flag, uint8_t data)
// {
//   switch (flag)
//   {
//   case I2C_DEV_OFFS:
//     // slave->dev.data_offs = data;
//     slave->dev.data_offs = 0;
//     slave->dev.data[slave->dev.data_offs] = data;
//     slave->dev.data_offs++;
//     break;
//   case I2C_DEV_DATA:
//     slave->dev.data[slave->dev.data_offs] = data;
//     slave->dev.data_offs++;
//     break;

//   default:
//     break;
//   }
// }

// static inline int32_t i2c_dev_address_check(struct i2c_slave *slave, int32_t val)
// {
//   if (slave->dev.addr == (val >> 1))
//     return I2C_RET_OK;
//   return I2C_RET_END;
// }

// static inline int32_t wait_for_scl(struct i2c_slave *slave, int32_t level)
// {
//   volatile uint32_t i = 3000;
//   while (i2c_scl_get() != level)
//   {
//     if (--i == 0)
//     {
//       slave->state = I2C_STATE_TIMEOUT;
//       return I2C_RET_END;
//     }
//   }
//   return I2C_RET_OK;
// }
// static int32_t slave_byte_write(struct i2c_slave *slave, uint8_t val)
// {
//   uint8_t i;
//   uint8_t data = 0x80;
  
//   for (i = 0; i < 8; i++)
//   {
//     i2c_sda_set(val & data);
//     // i2c_sda_set(0);
//     data = data >> 1;
//     if (wait_for_scl(slave, 1) == I2C_RET_END)
//     {
//       i2c_sda_set(1);
//       return I2C_RET_END;
//     }
//     if (wait_for_scl(slave, 0) == I2C_RET_END)
//     {
//       i2c_sda_set(1);
//       return I2C_RET_END;
//     }
//   }
//   i2c_sda_set(1);
//   return I2C_RET_OK;
// }

// static int slave_byte_read(struct i2c_slave *slave, uint8_t *data)
// {
//   uint8_t i;
//   volatile int32_t val, temp;
//   volatile int count;
//   for (i = 0x0; i < 0x8; i++)
//   {
//     //TODO:timeout check
//     if (wait_for_scl(slave, 1) != I2C_RET_OK)
//     {
//       return I2C_RET_END;
//     }
//     val = (val << 0x1) | i2c_sda_get();
//     count = 3000;
//     while (i2c_scl_get())
//     { //等待高点平结束，判断是否出现异常情况
//       //TODO:timeout check
//       /* sda is drivered by master now.
//        * if it changes when scl is high,stop or start happened */
//       if ((count--) == 0)
//       {
//         slave->state = I2C_STATE_TIMEOUT;
//         return I2C_RET_END;
//       }
//       temp = i2c_sda_get(); //读取数据线
//       if (!i2c_scl_get())   //当时钟线为低电平时退出循环
//         break;
//       if ((val & 1) != temp) //判断数据线是否发生了跳变
//       {
//         if (temp) //数据线被释放
//         {
//           slave->state = I2C_STATE_STOP;
//         }
//         else
//         {
//           slave->state = I2C_STATE_START;
//           wait_for_scl(slave, 0);
//         }
//         return I2C_RET_END;
//       }
//     }
//   }
//   *data = val;
//   return I2C_RET_OK;
// }
// static inline int i2c_ack_send(struct i2c_slave *slave)
// {
//   /* slave driver sda to low for ACK */
//   i2c_sda_set(0);
//   /* wait master read(scl rising edge trigger) ACK */
//   /* wait scl to HIGH */
//   if (wait_for_scl(slave, 1) != I2C_RET_OK)
//   {
//     i2c_sda_set(1);
//     return I2C_RET_END;
//   }
//   /* wait scl to low */
//   if (wait_for_scl(slave, 0) != I2C_RET_OK)
//   {
//     i2c_sda_set(1);
//     return I2C_RET_END;
//   }
//   i2c_sda_set(1);
//   return I2C_RET_OK;
// }

// static inline int32_t i2c_ack_read(struct i2c_slave *slave)
// {
//   volatile int32_t val, temp;
//   volatile int count;
//   /* wait master set sda */
//   if (wait_for_scl(slave, 1) == I2C_RET_END)
//   {
//     slave->state = I2C_STATE_STOP;
//     return I2C_RET_END;
//   }
//   count = 2000;
//   val = i2c_sda_get();
//   while (i2c_scl_get())
//   {
//     if ((count--) == 0)
//     {
//       slave->state = I2C_STATE_TIMEOUT;
//       return I2C_RET_END;
//     }
//     temp = i2c_sda_get();
//     if (!i2c_scl_get())
//       break;

//     if (val != temp)
//     {
//       if (temp)
//       {
//         slave->state = I2C_STATE_STOP;
//       }
//       else
//       {
//         slave->state = I2C_STATE_START;
//       }
//       return I2C_RET_END;
//     }
//   }
//   if (val == 0x0)
//   {
//     /* ACK */
//     slave->state = I2C_STATE_ACK;
//     return I2C_RET_OK;
//   }
//   else
//   { //如果没有应答，接收停止位
//     /* NACK */
//     slave->state = I2C_STATE_NACK;
//     /* wait master set sda */
//     if (wait_for_scl(slave, 1) == I2C_RET_END)
//     {
//       return I2C_RET_END;
//     }
//     count = 5000;
//     val = i2c_sda_get();
//     while (i2c_scl_get())
//     {
//       if ((count--) == 0)
//       {
//         slave->state = I2C_STATE_TIMEOUT;
//         return I2C_RET_END;
//       }
//       temp = i2c_sda_get();
//       if (!i2c_scl_get())
//         break;

//       if (val != temp)
//       {
//         if (temp)
//         {
//           slave->state = I2C_STATE_STOP;
//         }
//         else
//         {
//           slave->state = I2C_STATE_START;
//         }
//         return I2C_RET_END;
//       }
//     }
//     return I2C_RET_END;
//   }
// }
// static inline int32_t slave_data_send(struct i2c_slave *slave)
// {
//   uint8_t val;
//   do
//   {
//     val = buf_pop();
//     if (slave_byte_write(slave, val) == I2C_RET_END)
//     {
//       return I2C_RET_END;
//     }
//   } while (i2c_ack_read(slave) == I2C_RET_OK);
//   return I2C_RET_OK;
// }

// static inline int32_t slave_data_receive(struct i2c_slave *slave)
// {
//   // volatile int32_t val = 0;
//   uint8_t byte;
//   uint8_t flag = I2C_DEV_OFFS;
//   while (slave_byte_read(slave, &byte) != I2C_RET_END)
//   {
//     if (i2c_ack_send(slave) == I2C_RET_END)
//     {
//       return I2C_RET_END;
//     }

//     i2c_slave_store_data(slave, flag, byte);
//     flag = I2C_DEV_DATA;
//   }

//   return I2C_RET_OK;
// }
