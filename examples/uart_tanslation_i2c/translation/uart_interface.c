/**
 * @file uart_interface.c
 * @brief 
 * 
 * Copyright (c) 2021 Sipeed team
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

#include "hal_gpio.h"
#include "uart_interface.h"
#include "hal_usb.h"
#include "hal_dma.h"
#include "i2c_slave.h"
// #include "io_cfg.h"
#include"buff.h"
#include <string.h>

#define USB_OUT_RINGBUFFER_SIZE (8 * 1024)
#define UART_RX_RINGBUFFER_SIZE (8 * 1024)
#define UART_TX_DMA_SIZE (4095)

uint8_t usb_rx_mem[USB_OUT_RINGBUFFER_SIZE] __attribute__((section(".system_ram")));
// uint8_t uart_rx_mem[UART_RX_RINGBUFFER_SIZE] __attribute__((section(".system_ram")));

uint8_t src_buffer[UART_TX_DMA_SIZE] __attribute__((section(".tcm_code")));

struct device *uart1;
struct device *dma_ch2;

Ring_Buffer_Type usb_rx_rb;
// Ring_Buffer_Type uart1_rx_rb;



#define likely(x) __builtin_expect(!!(x), 1) //gcc内置函数, 帮助编译器分支优化
#define unlikely(x) __builtin_expect(!!(x), 0)
uint8_t s_l = 3;
uint8_t uart_flage = 0;
void uart_irq_callback(struct device *dev, void *args, uint32_t size, uint32_t state)
{
    static uint8_t num_t = 0;
    uint8_t *buf = (uint8_t*)args;
    for (int num = 0;num < size; num++)
    {
        switch (uart_flage)
        {
        case 0:
        {
            if(unlikely(buf[num] == 0x41))        //判断是否等于A
            {
                uart_flage = 10;
            }
            buf_push(buf[num]);  //透传缓冲区
        }
        break;
        case 10:    
        {
            if(likely(buf[num] == 0x54))       //判断是否等于T
            {
                uart_flage = 12;
            }
            else
            {
                uart_flage = 0;
            }
            buf_push(buf[num]);  //透传缓冲区
        }
        break;

        case 12:    
        {
            switch (buf[num])
            {
            case 0x53:      //判断是否等于S
                buf_clean(2);//透传缓冲区清2
                buf_switch(0);   //执行切换
                uart_flage = 30;
                break;
            case 0x43:      //判断是否等于C
                uart_flage = 15;
                buf_switch(0);   //执行切换
                break;
            // case 0x86:          //执行切换串口速率
            //     uart_flage = 100;
            //     break;
            default:
                uart_flage = 0;
                buf_push(buf[num]);  //透传缓冲区
                break;
            }
        }
        break;
        case 15:
        {
            if(likely(buf[num] == 0x41))        //判断是否等于A
            {
                uart_flage = 17;
            }
        }
        break;
        case 17:    
        {
            if(likely(buf[num] == 0x54))       //判断是否等于T
            {
                uart_flage = 20;
            }
            else
            {
                uart_flage = 15;
            }
        }
        break;
        case 20:    
        {
            if(buf[num] == 0x53)        //判断是否等于S
            {
                uart_flage = 30;
            }
            else if(buf[num] == 0x43)   //判断是否等于C
            {
                uart_flage = 15;
                buf_switch(0);   //执行切换
            }
            else
            {                       //进入at指令模式后，应该是主动切换出去
                buf_switch(1);   //执行切换
                uart_flage = 0;
            }
        }
        break;
        case 30:    
        {
            num_t = buf[num];
            uart_flage = 32;
        }
        break;
        case 32:    
        {
            if(likely(num_t --) ) buf_push(buf[num]); //AT缓冲区
            if(unlikely(num_t == 0)) uart_flage = 15;  
        }
        break;
        // case 100:
        // {
        //     uint8_t val_v = buf[num];
        //     switch (s_l)
        //     {
        //     case 3:
        //         baudrate = 0;
        //         baudrate |= val_v << s_l * 8;
        //         s_l--;
        //         break;
        //     case 1 ... 2:
        //         baudrate |= val_v << s_l * 8;
        //         s_l--;
        //         break;
        //     case 0:
        //         baudrate |= val_v ;
        //         s_l = 3;
        //         uart1_config(baudrate, 8, UART_PAR_NONE, UART_STOP_ONE);
        //         uart_flage = 0;
        //         break;
        //     default:
        //         uart_flage = 0;
        //         break;
        //     }
        // }
        // break;
        default:
            uart_flage = 0;
            break;
        }
    }
    memset(args, 0, size);
}

void uart1_init(void)
{
    uart_register(UART1_INDEX, "uart1", DEVICE_OFLAG_RDWR);
    uart1 = device_find("uart1");

    if (uart1)
    {
        device_open(uart1, DEVICE_OFLAG_DMA_TX | DEVICE_OFLAG_INT_RX); //uart0 tx dma mode
        device_control(uart1, DEVICE_CTRL_SUSPEND, NULL);
        device_set_callback(uart1, uart_irq_callback);
        device_control(uart1, DEVICE_CTRL_SET_INT, (void *)(UART_RX_FIFO_IT | UART_RTO_IT));
    }

    dma_register(DMA0_CH2_INDEX, "ch2", DEVICE_OFLAG_RDWR);
    dma_ch2 = device_find("ch2");
    if (dma_ch2)
    {
        device_open(dma_ch2, 0);
    }
}

void uart1_config(uint32_t baudrate, uart_databits_t databits, uart_parity_t parity, uart_stopbits_t stopbits)
{
    uart_param_cfg_t cfg;
    cfg.baudrate = baudrate;
    cfg.stopbits = stopbits;
    cfg.parity = parity;

    if (databits == 5)
    {
        cfg.databits = UART_DATA_LEN_5;
    }
    else if (databits == 6)
    {
        cfg.databits = UART_DATA_LEN_6;
    }
    else if (databits == 7)
    {
        cfg.databits = UART_DATA_LEN_7;
    }
    else if (databits == 8)
    {
        cfg.databits = UART_DATA_LEN_8;
    }

    device_control(uart1, DEVICE_CTRL_CONFIG, &cfg);
}

void ringbuffer_lock()
{
    disable_irq();
}
void ringbuffer_unlock()
{
    enable_irq();
}

void uart_ringbuffer_init(void)
{
    /* init mem for ring_buffer */
    memset(usb_rx_mem, 0, USB_OUT_RINGBUFFER_SIZE);
    // memset(uart_rx_mem, 0, UART_RX_RINGBUFFER_SIZE);

    /* init ring_buffer */
    Ring_Buffer_Init(&usb_rx_rb, usb_rx_mem, USB_OUT_RINGBUFFER_SIZE, ringbuffer_lock, ringbuffer_unlock);
    // Ring_Buffer_Init(&uart1_rx_rb, uart_rx_mem, UART_RX_RINGBUFFER_SIZE, ringbuffer_lock, ringbuffer_unlock);
}

static dma_control_data_t uart_dma_ctrl_cfg =
    {
        .bits.fix_cnt = 0,
        .bits.dst_min_mode = 0,
        .bits.dst_add_mode = 0,
        .bits.SI = 1,
        .bits.DI = 0,
        .bits.SWidth = DMA_TRANSFER_WIDTH_8BIT,
        .bits.DWidth = DMA_TRANSFER_WIDTH_8BIT,
        .bits.SBSize = 0,
        .bits.DBSize = 0,
        .bits.I = 0,
        .bits.TransferSize = 4095};
static dma_lli_ctrl_t uart_lli_list =
    {
        .src_addr = (uint32_t)src_buffer,
        .dst_addr = DMA_ADDR_UART1_TDR,
        .nextlli = 0};

extern void led_toggle(uint8_t idx);
void uart_send_from_ringbuffer(void)
{
    if (Ring_Buffer_Get_Length(&usb_rx_rb))
    {
        if (!device_control(dma_ch2, DMA_CHANNEL_GET_STATUS, NULL))
        {
            uint32_t avalibleCnt = Ring_Buffer_Read(&usb_rx_rb, src_buffer, UART_TX_DMA_SIZE);

            if (avalibleCnt)
            {
                dma_channel_stop(dma_ch2);
                uart_dma_ctrl_cfg.bits.TransferSize = avalibleCnt;
                memcpy(&uart_lli_list.cfg, &uart_dma_ctrl_cfg, sizeof(dma_control_data_t));
                device_control(dma_ch2, DMA_CHANNEL_UPDATE, (void *)((uint32_t)&uart_lli_list));
                dma_channel_start(dma_ch2);
            }
        }
    }
}