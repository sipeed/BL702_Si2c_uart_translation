// /*
//  * i2c_slave.c
//  *
//  *  Created on: Jul 31, 2017
//  *      Author: epanda
//  */

// #include <stdint.h>
// #include <stdlib.h>

// #include "hal_gpio.h"
// #include "i2c_slave.h"



// #define sda_io GPIO_PIN_0 
// #define scl_io GPIO_PIN_15


// // #define gpio_w32(b, off)  do { } while (0)
// // #define gpio_r32(off)   do { } while (0)

// #define I2C_STATE_NACK     1
// #define I2C_STATE_ACK      0
// #define I2C_STATE_IDLE    -1
// #define I2C_STATE_START   -2
// #define I2C_STATE_STOP    -3
// #define I2C_STATE_DEVICE  -4
// #define I2C_STATE_TIMEOUT -5

// #define I2C_RET_OK     0
// #define I2C_RET_END    -1

// #define I2C_DEV_OFFS  0x0
// #define I2C_DEV_DATA  0x1

// static inline int32_t slave_data_receive(struct i2c_slave *slave);
// static inline int32_t slave_data_send(struct i2c_slave *slave);
// static inline int32_t slave_byte_read(struct i2c_slave *slave);
// static inline int32_t slave_byte_write(struct i2c_slave *slave, uint8_t val);
// static inline int32_t i2c_dev_address_check(struct i2c_slave *slave, int32_t val);
// static inline void i2c_ack_send(struct i2c_slave *slave);
// static inline int32_t i2c_ack_read(struct i2c_slave *slave);
// static inline void i2c_pins_init(struct i2c_slave *slave);
// static inline int32_t i2c_scl_get(struct i2c_slave *slave);
// static inline int32_t i2c_sda_get(struct i2c_slave *slave);
// static inline void i2c_sda_set(struct i2c_slave *slave, int32_t val);
// static inline int32_t wait_for_scl(struct i2c_slave *slave, int32_t level);
// static inline int32_t gpio_value_get(uint32_t gpio);
// static inline void gpio_direction_input(uint32_t gpio);
// static inline void gpio_direction_output(uint32_t gpio, int32_t val);
// static inline void delay(uint32_t count);


// static void sda_interrupt_callback(uint32_t pin);


// struct i2c_slave *soft_i2c_slave = NULL;

// // struct i2c_slave nihao;
// int32_t i2c_slave_init(void)
// {
//   uint32_t i, j;
  
  
//   uint8_t slave_addr[I2C_SLAVE_DEV_MAX] =
//   {
//     /* 7 bits address */
//     0x14 >> 0x1
//   };

//   soft_i2c_slave = malloc(sizeof(struct i2c_slave));

//   // soft_i2c_slave = &nihao;

//   if(!soft_i2c_slave)
//   {
//     return -1;
//     gpio_write(GPIO_PIN_17, 0);
//   }

//   for(j = 0; j < I2C_SLAVE_DEV_MAX; j++)
//   {
//     soft_i2c_slave->dev[j].data_offs = 0;
//     soft_i2c_slave->dev[j].addr = slave_addr[j];
//     for(i = 0; i < sizeof(soft_i2c_slave->dev[0].data); i++)
//     {
//       soft_i2c_slave->dev[j].data[i] = 0;
//     }
//   }

//   soft_i2c_slave->sda = sda_io;
//   soft_i2c_slave->scl = scl_io;

//   i2c_pins_init(soft_i2c_slave);
//   return 0;
// }



// static void sda_interrupt_callback(uint32_t pin)
// {
//   int32_t val;
//   uint32_t idx;
//   gpio_irq_enable(soft_i2c_slave->sda,DISABLE);//关闭中断
//   gpio_set_mode(soft_i2c_slave->sda,GPIO_INPUT_MODE);   //将sda设置成输入

//   if(wait_for_scl(soft_i2c_slave, 0) == I2C_RET_END)
//   {
//     /* timeout */
//     goto end;
//   }

//   soft_i2c_slave->state = I2C_STATE_START;

//   while(soft_i2c_slave->state == I2C_STATE_START)
//   {
//     /* read address + R/W bit */
//     val = slave_byte_read(soft_i2c_slave);
//     if(val == I2C_RET_END)
//     {
//       if(soft_i2c_slave->state == I2C_STATE_START)
//       {
//         continue;
//       }
//       /* timeout, stop */
//       goto end;
//     }

//     idx = i2c_dev_address_check(soft_i2c_slave, val);
//     if(idx == I2C_RET_END)
//     {
//       /* device address mismatch */
//       goto end;
//     }

//     soft_i2c_slave->state = I2C_STATE_DEVICE;

//     /* send ACK */
//     i2c_ack_send(soft_i2c_slave);

//     soft_i2c_slave->dev_idx = idx;

//     /* check R/W bit */
//     if(val & 1)
//     {                       // 主机读操作
//       //TODO: bug.master can read data without pre-send device data offset
//       val = slave_data_send(soft_i2c_slave);
//       if(val == I2C_RET_END)
//       {
//         if(soft_i2c_slave->state == I2C_STATE_START)
//         {
//           continue;
//         }
//         else
//         {
//           goto end;
//         }
//       }
//     }
//     else
//     {                     //主机写
//       val = slave_data_receive(soft_i2c_slave);
//       if(val == I2C_RET_END)
//       {
//         if(soft_i2c_slave->state == I2C_STATE_START)
//         {
//           continue;
//         }
//         else
//         {
//           goto end;
//         }
//       }
//     }
//   }

//   /* wait scl and sda high */
//   while (!(i2c_scl_get(soft_i2c_slave) && i2c_sda_get(soft_i2c_slave)));
//   soft_i2c_slave->state = I2C_STATE_IDLE;
//   i2c_pins_init(soft_i2c_slave);

//   i2c_date_commit();
//   return ;

// end:
//   i2c_pins_init(soft_i2c_slave);
//   soft_i2c_slave->state = I2C_STATE_IDLE;
//   return ;
// }
// // static inline int32_t i2c_slave_sda_interrupt_callback(struct i2c_slave *slave)
// // {

// // }

// static inline int32_t i2c_scl_get(struct i2c_slave *slave)
// {
//   // int32_t val = gpio_value_get(slave->scl);
//   // return val;‘
//   return gpio_read(slave->scl);
// }

// static inline void i2c_sda_set(struct i2c_slave *slave, int32_t val)
// {
//   if (val)
//     gpio_set_mode(slave->sda,GPIO_INPUT_MODE);
//     // gpio_direction_input(slave->sda);
//   else
//   {
//     gpio_set_mode(slave->sda,GPIO_OUTPUT_PD_MODE);    //设置开漏输出
//     gpio_write(slave->sda, val);
//   }
//     // gpio_direction_output(slave->sda, 0);
// }

// static inline int32_t i2c_sda_get(struct i2c_slave *slave)
// {
//   return gpio_read(slave->sda);
//   // int32_t val = gpio_value_get(slave->sda);
//   // return val;
// }

// static inline void i2c_pins_init(struct i2c_slave *slave)
// {
//   // gpio_direction_input(slave->scl);                             //时钟输入
//   gpio_set_mode(slave->scl,GPIO_INPUT_MODE);                    //时钟输入
// /*-----------------------------------------------------------------------------------------------------*/
//   // gpio_direction_input(slave->sda);
//   gpio_set_mode(slave->sda,GPIO_SYNC_FALLING_TRIGER_INT_MODE);             //从下降沿中断
//   gpio_attach_irq(slave->sda,sda_interrupt_callback);
//   gpio_irq_enable(slave->sda,ENABLE);

// }

// static inline void gpio_direction_input(uint32_t gpio)
// {
//   /*-----------------------------------------------------------------------------------------------------*/
//   gpio_set_mode(gpio,GPIO_INPUT_MODE);
// }

// static inline void gpio_direction_output(uint32_t gpio, int val)
// {
//   // gpio_value_set(gpio, val);
//   gpio_set_mode(gpio,GPIO_OUTPUT_PD_MODE);    //设置开漏输出
//   gpio_write(gpio, val);
//   // gpio_value_set(gpio, val);
// }

// static inline int32_t gpio_value_get(uint32_t gpio)
// {
//   return gpio_read(gpio);
// }

// // static inline void gpio_value_set(uint32_t gpio, int32_t val)
// // {
// //   gpio_set_mode(gpio,GPIO_OUTPUT_PD_MODE);    //设置开漏输出
// //   gpio_value_set(gpio, val);

// //   // (void)gpio;
// //   // (void)val;
// // }

// static void i2c_slave_store_data(struct i2c_slave *slave, uint8_t flag, uint8_t data)
// {
//   switch(flag)
//   {
//     case I2C_DEV_OFFS:
//       slave->dev[slave->dev_idx].data_offs = data;
//       break;

//     case I2C_DEV_DATA:
//       slave->dev[slave->dev_idx].data[slave->dev[slave->dev_idx].data_offs] = data;
//       slave->dev[slave->dev_idx].data_offs++;
//       break;

//     default:
//       break;
//   }
// }

// static inline int32_t i2c_dev_address_check(struct i2c_slave *slave, int32_t val)
// {
//   uint32_t i;
//   for(i = 0; i < I2C_SLAVE_DEV_MAX; i++)
//   {
//     if(slave->dev[i].addr == (val>>1))
//       return i;
//   }
//   return -1;
// }

// static inline void delay(uint32_t count)  //延时us
// {
//   bflb_platform_delay_us(count);
// }

// static inline int32_t wait_for_scl(struct i2c_slave *slave, int32_t level)
// {
//   uint32_t i = 2000;        //2000us
//   while(i2c_scl_get(slave) != level)
//   {
//     if(--i == 0)
//     {
//       slave->state = I2C_STATE_TIMEOUT;
//       return I2C_RET_END;
//     }
//     delay(1);
//   }
//   return I2C_RET_OK;
// }

// static inline int32_t slave_byte_write(struct i2c_slave *slave, uint8_t val)
// {
//   uint8_t i;

//   for(i = 0; i < 8; i++)
//   {
//     i2c_sda_set(slave, val & (1 << (7-i)));
//     if(wait_for_scl(slave, 1) == I2C_RET_END)
//     {
//       return I2C_RET_END;
//     }
//     if(wait_for_scl(slave, 0) == I2C_RET_END)
//     {
//       return I2C_RET_END;
//     }
//   }
//   i2c_sda_set(slave, 1);
//   return I2C_RET_OK;
// }

// static inline int32_t slave_byte_read(struct i2c_slave *slave)
// {
//   uint8_t i;
//   int32_t val = 0;
//   int32_t temp;

//   for(i = 0x0; i < 0x8; i++)
//   {
//     while(!i2c_scl_get(slave))  //等待高电平
//     {
//       //TODO:timeout check
//     }

//     val = (val<<0x1) | i2c_sda_get(slave);    //读取位信号
//     while(i2c_scl_get(slave))   //等待低电平
//     {
//       //TODO:timeout check
//       /* sda is drivered by master now.
//        * if it changes when scl is high,stop or start happened */
//       temp = i2c_sda_get(slave);    //读到低电平
//       if(!i2c_scl_get(slave))   //如果是低电平就返回
//         break;
//       if((val & 1) != temp)     //
//       {
//         if(temp)
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
//   }
//   //return I2C_RET_OK;
//   return val;
// }

// static inline void i2c_ack_send(struct i2c_slave *slave)
// {
//   /* slave driver sda to low for ACK */
//   i2c_sda_set(slave, 0);
//   /* wait master read(scl rising edge trigger) ACK */
//   while(!i2c_scl_get(slave))//等待高电平
//   {
//     //TODO:timeout check
//   }

//   /* wait scl to low */
//   while(i2c_scl_get(slave)) //等待低电平
//   {
//     //TODO:timeout check
//   }
//   /* slave release sda */
//   i2c_sda_set(slave, 1);
// }

// static inline int32_t i2c_ack_read(struct i2c_slave *slave)
// {
//   int32_t val, temp;

//   /* wait master set sda */
//   while(!i2c_scl_get(slave))
//   {
//     //TODO:timeout check
//   }
//   /* read ACK */
//   val = i2c_sda_get(slave);
//   /* keep checking sda when scl high */
//   while(i2c_scl_get(slave))
//   {
//     //TODO:timeout check
//     /* sda is drivered by master now.
//      * if it changes when scl is high,stop or start happened */
//     temp = i2c_sda_get(slave);
//     if(!i2c_scl_get(slave))
//       break;

//     if(val != temp)
//     {
//       if(temp)
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

//   if(val == 0x0)
//   {
//     /* ACK */
//     return I2C_RET_OK;
//   }
//   else
//   {
//     /* NACK */
//     return I2C_RET_END;
//   }
// }

// static inline int32_t slave_data_send(struct i2c_slave *slave)
// {
//   uint8_t val, offs;

//   offs = slave->dev[slave->dev_idx].data_offs;
//   do
//   {
//     val = slave->dev[slave->dev_idx].data[offs];
//     offs ++;
//     if (slave_byte_write(slave, val) == I2C_RET_END)
//     {
//       return I2C_RET_END;
//     }
//   } while (i2c_ack_read(slave) == I2C_RET_OK);

//   slave->dev[slave->dev_idx].data_offs = 0;
//   slave->state = I2C_STATE_IDLE;
//   return I2C_RET_OK;
// }

// static inline int32_t slave_data_receive(struct i2c_slave *slave)
// {
//   int32_t val = 0;
//   uint8_t flag = I2C_DEV_OFFS;

//   do
//   {
//     val = slave_byte_read(slave);
//     if (val == I2C_RET_END)
//     {
//       return I2C_RET_END;
//     }

//     i2c_ack_send(slave);
//     i2c_slave_store_data(slave, flag, val);
//     flag = I2C_DEV_DATA;
//   } while (val != I2C_RET_END);

//   return I2C_RET_OK;
// }
