/**
 * @file main.c
 * @brief
 *
 * Copyright (c) 2021 Bouffalolab team
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 */

#include "hal_uart.h"
#include "hal_gpio.h"
#include "i2c_slave.h"
#include "hal_gpio.h"

#include "uart_interface.h"


#define sda_io GPIO_PIN_0 
#define scl_io GPIO_PIN_15
// #define send_MAX    10
// #define receive_MAX 10

// #define addrr 0x14


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

int i2c_flages;

//    pOut=(uint32_t *)(0x40000180);
int main(void)
{

    GLB_Select_Internal_Flash();

    bflb_platform_init(0);

    uart_ringbuffer_init();
    uart1_init();
    uart1_config(115200,8,UART_PAR_NONE,UART_STOP_ONE);
    RX_Data_Init();

    i2c_slave_init();



    gpio_set_mode(GPIO_PIN_1,GPIO_OUTPUT_PP_MODE);
    gpio_set_mode(GPIO_PIN_9,GPIO_OUTPUT_PP_MODE);
    gpio_set_mode(GPIO_PIN_17,GPIO_OUTPUT_PP_MODE);

    gpio_write(GPIO_PIN_9, 1);
    gpio_write(GPIO_PIN_17, 1);


    for(;;)
    {
        uart_send_from_ringbuffer();
        if(SDA_INPUT == 0) i2c_slave_sda_interrupt_callback();
        if(i2c_flages == 1)
        {
             
                uint8_t TX_AABB[4]={0xAA,0xBB,0,0};
                TX_AABB[2]=my_slave.dev.data[0];
                TX_AABB[3]=my_slave.dev.data[1];
                Ring_Buffer_Write(&usb_rx_rb,(uint8_t *)TX_AABB, 4);

                i2c_flages = 0;

        }
    }

}
