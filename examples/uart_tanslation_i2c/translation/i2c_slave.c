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

#define sda_io GPIO_PIN_0 
#define scl_io GPIO_PIN_15


#define io1_HIGH    ((*(volatile uint32_t *)0x40000188) |= (1<<1))
#define io1_LOW     ((*(volatile uint32_t *)0x40000188) &= (~(1<<1)))

#define SDA_HIGH    ((*(volatile uint32_t *)0x40000188) |= (1<<sda_io))
#define SDA_LOW     ((*(volatile uint32_t *)0x40000188) &= (~(1<<sda_io)))
#define SCL_HIGH    ((*(volatile uint32_t *)0x40000188) |= (1<<scl_io))
#define SCL_LOW     ((*(volatile uint32_t *)0x40000188) &= (~(1<<scl_io)))


#define SDA_da      ((*(volatile uint32_t *)0x40000180) & 1)
#define SCL_da      ((*(volatile uint32_t *)0x40000180) & 1 << 15)

//#define SDA_INPUT   ((*(volatile uint32_t *)0x40000180) & 1)
#define SDA_INPUT   SDA_da

// #define SCL_INPUT   ((*(volatile uint32_t *)0x40000180) & 1 << 15)
#define SCL_INPUT   SCL_da >> 15


// #define SDA_OUT     ((*(volatile uint32_t *)0x40000190) |= (1<<sda_io))
// #define SDA_IN      ((*(volatile uint32_t *)0x40000190) &= (~(1<<sda_io)))
// #define SCL_OUT     ((*(volatile uint32_t *)0x40000190) |= (1<<scl_io))
// #define SCL_IN      ((*(volatile uint32_t *)0x40000190) &= (~(1<<scl_io)))


#define SDA_OUT     gpio_set_mode(sda_io, GPIO_OUTPUT_PP_MODE)
#define SDA_IN      gpio_set_mode(sda_io, GPIO_INPUT_MODE)
#define SCL_OUT     gpio_set_mode(scl_io, GPIO_OUTPUT_PP_MODE)
#define SCL_IN      gpio_set_mode(scl_io, GPIO_INPUT_MODE)



#define i2c_slave_addr 0x14


#define I2C_STATE_NACK     1
#define I2C_STATE_ACK      0
#define I2C_STATE_IDLE    -1
#define I2C_STATE_START   -2
#define I2C_STATE_STOP    -3
#define I2C_STATE_DEVICE  -4
#define I2C_STATE_TIMEOUT -5

#define I2C_RET_OK     0
#define I2C_RET_END    -1

#define I2C_DEV_OFFS  0x0
#define I2C_DEV_DATA  0x1

static inline int32_t slave_data_receive(struct i2c_slave *slave) __attribute__((optimize(gcc_good)));
static inline int32_t slave_data_send(struct i2c_slave *slave) __attribute__((optimize(gcc_good)));
static inline int slave_byte_read(struct i2c_slave *slave ,uint8_t *data) __attribute__((optimize(gcc_good)));
static inline int32_t slave_byte_write(struct i2c_slave *slave, uint8_t val) __attribute__((optimize(gcc_good)));
static inline int32_t i2c_dev_address_check(struct i2c_slave *slave, int32_t val) __attribute__((optimize(gcc_good)));
static inline void i2c_ack_send(struct i2c_slave *slave) __attribute__((optimize(gcc_good)));
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
  uint32_t i, j;
  struct i2c_slave *slave;

  slave = &my_slave;
  slave->dev.addr = i2c_slave_addr;

  slave->dev.data_offs=0;

  for(i = 0; i < sizeof(slave->dev.data); i++)
  {
    slave->dev.data[i] = 0;
  }
  i2c_pins_init();
  return 0;
}

static once_count=0;


#define BUFFER_MAX  2560      //缓冲区大小

typedef struct _circle_buffer{
    uint8_t head_pos;             //缓冲区头部位置
    uint8_t tail_pos;             //缓冲区尾部位置   
    uint8_t circle_buffer[BUFFER_MAX];    //缓冲区数组 
}circle_buffer;

circle_buffer buffer;

void bufferPop(uint8_t* _buf)
{
    if(buffer.head_pos==buffer.tail_pos)        //如果头尾接触表示缓冲区为空
        *_buf=0x00;
    else
    {
        *_buf=buffer.circle_buffer[buffer.head_pos];    //如果缓冲区非空则取头节点值并偏移头节点
        if(++buffer.head_pos>=BUFFER_MAX)
            buffer.head_pos=0;
    }
}

void bufferPush(const uint8_t _buf)
{   
    buffer.circle_buffer[buffer.tail_pos]=_buf; //从尾部追加
    if(++buffer.tail_pos>=BUFFER_MAX)           //尾节点偏移
        buffer.tail_pos=0;                      //大于数组最大长度 制零 形成环形队列
        if(buffer.tail_pos==buffer.head_pos)    //如果尾部节点追到头部节点 则修改头节点偏移位置丢弃早期数据
        if(++buffer.head_pos>=BUFFER_MAX)
            buffer.head_pos=0;

}

uint32_t i2c_send_data(uint8_t send_data)
{
  struct i2c_slave *slave;
  slave=&my_slave;
  bufferPush(send_data);
  // slave->dev.send_data[once_count++]=send_data;
}


int32_t i2c_slave_sda_interrupt_callback()
{
  int32_t val;
  uint8_t byte;
  struct i2c_slave *slave;
  int32_t idx;
  slave = &my_slave;

  if(wait_for_scl(slave, 0) == I2C_RET_END)
  {
    /* timeout */
    goto end;
  }

  slave->state = I2C_STATE_START;

  while(slave->state == I2C_STATE_START)
  {
    /* read address + R/W bit */
    val = slave_byte_read(slave,&byte);
    
    if(val == I2C_RET_END)
    {
      if(slave->state == I2C_STATE_START)
      {
        continue;
      }
      /* timeout, stop */
      goto end;
    }

    idx = i2c_dev_address_check(slave, byte);

    if(idx == I2C_RET_END)
    {
      /* device address mismatch */
      goto end;
    }

    slave->state = I2C_STATE_DEVICE;

    /* send ACK */

    i2c_ack_send(slave);
    /* check R/W bit */
    val = byte;
    if(val & 1)
    {
      //TODO: bug.master can read data without pre-send device data offset
      val = slave_data_send(slave);
      if(val == I2C_RET_END)
      {
        if(slave->state == I2C_STATE_START)
        {
          continue;
        }
        else
        {
          
          goto end;
        }
      }
    }
    else
    {
      val = slave_data_receive(slave);
      if(val == I2C_RET_END)
      {
        if(slave->state == I2C_STATE_START)
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
  }

  /* wait scl and sda high */
  while (!(i2c_scl_get() && i2c_sda_get()));
  slave->state = I2C_STATE_IDLE;
  // MSG("i receiveone:%x01",byte);
  return 0;

end:
  i2c_pins_init();
  slave->state = I2C_STATE_IDLE;
  MSG("i receivetwo:%01x \r\n",byte);
  return -1;
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
  switch(flag)
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
    if(slave->dev.addr == (val>>1))return 0;
      
  return -1;
}



static inline int32_t wait_for_scl(struct i2c_slave *slave, int32_t level)
{
  uint32_t i = 1000;
  while(i2c_scl_get() != level)
  {
    if(--i == 0)
    {
      slave->state = I2C_STATE_TIMEOUT;
      return I2C_RET_END;
    }
  }
  return I2C_RET_OK;
}

static inline int32_t slave_byte_write(struct i2c_slave *slave, uint8_t val)
{
  uint8_t i;

  for(i = 0; i < 8; i++)
  {
    i2c_sda_set(val & (1 << (7-i)));
    if(wait_for_scl(slave, 1) == I2C_RET_END)
    {
      return I2C_RET_END;
    }
    if(wait_for_scl(slave, 0) == I2C_RET_END)
    {
      return I2C_RET_END;
    }
  }
  i2c_sda_set(1);
  return I2C_RET_OK;
}

static inline int slave_byte_read(struct i2c_slave *slave ,uint8_t *data)
{
  uint8_t i;
  int32_t val, temp;
  int count;

  for(i = 0x0; i < 0x8; i++)
  {
    count = 1000;
    while(!i2c_scl_get())
    {
      //TODO:timeout check
      if((count --) == 0)
      {
        return I2C_RET_END;
      }
    }

    val = (val<<0x1) | i2c_sda_get();
    while(i2c_scl_get())
    {
      //TODO:timeout check
      /* sda is drivered by master now.
       * if it changes when scl is high,stop or start happened */
      temp = i2c_sda_get();
      if(!i2c_scl_get())
        break;
      if((val & 1) != temp)
      {
        if(temp)
        {
          slave->state = I2C_STATE_STOP;
          io1_HIGH;
          io1_LOW;
        }
        else
        {
          slave->state = I2C_STATE_START;
        }
        return I2C_RET_END;
      }
    }
  }
  *data = val;
  return I2C_RET_OK;
}

static inline void i2c_ack_send(struct i2c_slave *slave)
{
  int count;
  /* slave driver sda to low for ACK */
  i2c_sda_set(0);
  /* wait master read(scl rising edge trigger) ACK */
  count = 1000;
  while(!i2c_scl_get())
  {
    //TODO:timeout check
    if((count --) == 0)                                                                                                               
    {
      goto end_ack;
    }
  }

  /* wait scl to low */
  count = 1000;
  while(i2c_scl_get())
  {
    //TODO:timeout check
    if((count --) == 0)                                                                                                               
    {
      goto end_ack;
    }
  }
end_ack:
  /* slave release sda */
  i2c_sda_set(1);
}

static inline int32_t i2c_ack_read(struct i2c_slave *slave)
{
  int32_t val, temp;
  int count;

  /* wait master set sda */
  count = 1000;
  while(!i2c_scl_get())
  {
    //TODO:timeout check
    if((count --) == 0)                                                                                                               
    {
      return I2C_RET_END;
    }
  }
  /* read ACK */
  val = i2c_sda_get();
  /* keep checking sda when scl high */
  while(i2c_scl_get())
  {
    //TODO:timeout check
    /* sda is drivered by master now.
     * if it changes when scl is high,stop or start happened */
    temp = i2c_sda_get();
    if(!i2c_scl_get())
      break;

    if(val != temp)
    {
      if(temp)
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

  if(val == 0x0)
  {
    /* ACK */
    return I2C_RET_OK;
  }
  else
  {
    /* NACK */
    return I2C_RET_END;
  }
}


/*
  i2c发送入口
  发送数据
*/
static inline int32_t slave_data_send(struct i2c_slave *slave)
{
  uint8_t val, offs;

  // for(int i = 0;i<9;i++)
  // {
  //   if (slave_byte_write(slave, slave->dev.send[i]) == I2C_RET_END)
  //   {
  //     return I2C_RET_END;
  //   }
  //   if(i2c_ack_read(slave) != I2C_RET_OK)
  //   {
  //     break;
  //   }
  // }
  // offs = slave->dev.data_offs;
  offs = 0;
  do
  {
    // struct i2c_slave *slave;
    // slave=&my_slave;
    bufferPop(&val);
    // val = slave->dev.send_data[offs];
    offs ++;
    if (slave_byte_write(slave, val) == I2C_RET_END)
    {
      return I2C_RET_END;
    }
  } while (i2c_ack_read(slave) == I2C_RET_OK);

  once_count=0;
  memset(slave->dev.send_data,0,256);
  slave->dev.data_offs = 0;
  slave->state = I2C_STATE_IDLE;
  return I2C_RET_OK;
}
static inline int32_t slave_data_receive(struct i2c_slave *slave)
{
  int32_t val = 0;
  uint8_t byte;
  uint8_t flag = I2C_DEV_OFFS;
  // for(int i =0 ;i<2;i++)
  // {
  //   val = slave_byte_read(slave,&byte);
  //   if (val == I2C_RET_END)
  //   {
  //     return I2C_RET_END;
  //   }
  //   i2c_ack_send(slave);
  //   slave->dev.receive[i] = byte;
  // }
  do
  {
    val = slave_byte_read(slave,&byte);
    if (val == I2C_RET_END)
    {
      return I2C_RET_END;
    }

    i2c_ack_send(slave);
    i2c_slave_store_data(slave, flag, byte);
    flag = I2C_DEV_DATA;
  } while (val != I2C_RET_END);
  return I2C_RET_OK;
}