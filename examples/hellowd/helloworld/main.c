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
#include "uart_interface.h"
#include "i2c_slave.h"
#include "hal_gpio.h"


//extern Ring_Buffer_Type usb_rx_rb;
char nihao[50] = "nihao\r\n" ;
extern struct i2c_slave *soft_i2c_slave;

int k = 0;
void i2c_date_commit()
{
    // int mk;
    // mk = soft_i2c_slave->dev[soft_i2c_slave->dev_idx].data_offs;
    // Ring_Buffer_Write(&usb_rx_rb,soft_i2c_slave->dev[soft_i2c_slave->dev_idx].data, mk);
    soft_i2c_slave->dev[soft_i2c_slave->dev_idx].data_offs=0;
    gpio_write(GPIO_PIN_17,k);
    if (k == 1)
    {
        k=0;
    }
    else
    {
        k=1;
    }
    
}




int main(void)
{
    int conut = 0;
    GLB_Select_Internal_Flash();

    bflb_platform_init(0);
    uart_ringbuffer_init();
    uart1_init();
    
    uart1_config(115200,8,UART_PAR_NONE,UART_STOP_ONE);
    // i2c_slave_init();
    gpio_set_mode(GPIO_PIN_17,GPIO_OUTPUT_PP_MODE);
    gpio_write(GPIO_PIN_17, 1);
    while (1) {
        MSG("hello world!\r\n");
        uart_send_from_ringbuffer();
        // MSG("status:%s",print_buf);
        gpio_write(GPIO_PIN_17, 1);
        bflb_platform_delay_ms(10);
        gpio_write(GPIO_PIN_17, 0);
        bflb_platform_delay_ms(10);
        conut ++;
        if(conut % 100 == 0){
            Ring_Buffer_Write(&usb_rx_rb,nihao, 7);
        }
        
        


    }
}
