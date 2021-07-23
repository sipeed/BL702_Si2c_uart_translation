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

#define USB_OUT_RINGBUFFER_SIZE (8 * 1024)
#define UART_RX_RINGBUFFER_SIZE (8 * 1024)
#define UART_TX_DMA_SIZE (4095)

uint8_t usb_rx_mem[USB_OUT_RINGBUFFER_SIZE] __attribute__((section(".system_ram")));
uint8_t uart_rx_mem[UART_RX_RINGBUFFER_SIZE] __attribute__((section(".system_ram")));

uint8_t src_buffer[UART_TX_DMA_SIZE] __attribute__((section(".tcm_code")));

struct device* uart1;
struct device* dma_ch2;

Ring_Buffer_Type usb_rx_rb;
Ring_Buffer_Type uart1_rx_rb;

struct
{
    uint8_t URD_Count;
    uint8_t UART_pData[256];
    uint8_t UART_RX_State;
    /* data */
}UART_RX;

void RX_Data_Init(void)
{
    UART_RX.URD_Count=0;
    UART_RX.UART_RX_State=0;

    memset(UART_RX.UART_pData,0,256);
    
}

int uart_status;
void uart_irq_callback(struct device *dev, void *args, uint32_t size, uint32_t state)
{
    memcpy(UART_RX.UART_pData,(uint8_t *)args,size);
    memset(args,0,size);
    // gpio_write(GPIO_PIN_9, 0);
    // if(UART_RX.UART_pData[0]==0xBB && UART_RX.UART_pData[1]==0xAA)
    // {
        
    int i;
    for(i=0;i<size;i++)
    {
        i2c_send_data(UART_RX.UART_pData[i]);
        
    } 
    // }
    RX_Data_Init();
}

    // int i;
    // for(i=0;i<size;i=i+11)
    // {

    //     if(UART_RX.UART_pData[i]==0xBB)
    //     {
    //         if(UART_RX.UART_pData[i+1]==0xAA)
    //         {
    //             int j;
    //             for(j=0;j<9;j++)
    //             {
    //                 i2c_send_data(UART_RX.UART_pData[j+i+2]);
    //             }
    //         }
    //     }


    // }
    // if(UART_RX.UART_pData[0]==0xBB)
    // {
    //     if(UART_RX.UART_pData[1]==0xAA)
    //     {
            

    //         // for(queue_count=0;queue_count<size-2;queue_count++)
    //         // {
    //         //     // insert_queue(myqueue, UART_RX.UART_pData[2+queue_count]);
    //         // }
            
    //         // queue_count--;
    //         // delete_queue(myqueue);


    //         gpio_write(GPIO_PIN_9, 0);
    //         for(int i=0;i<9;i++)
    //         {
    //             // UART_RX.UART_RX_Data[i]=UART_RX.UART_pData[2+i];
    //             i2c_send_data(UART_RX.UART_pData[2+i],i);
    //         }

    //         // Ring_Buffer_Write(&usb_rx_rb,(uint8_t *)UART_RX.UART_RX_Data, 9);

    //         // i2c_Buffer_Write(*UART_RX.UART_RX_Data);
    //         // UART_flages=1;
    //         RX_Data_Init();
    //         // gpio_write(GPIO_PIN_17, 0);    
    //     }

    
        
    
    

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
        //device_set_callback(dma_ch2, NULL);
        //device_control(dma_ch2, DEVICE_CTRL_SET_INT, NULL);
    }
    //device_control(uart1, DEVICE_CTRL_ATTACH_TX_DMA, dma_ch2);

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
    memset(uart_rx_mem, 0, UART_RX_RINGBUFFER_SIZE);

    /* init ring_buffer */
    Ring_Buffer_Init(&usb_rx_rb, usb_rx_mem, USB_OUT_RINGBUFFER_SIZE, ringbuffer_lock, ringbuffer_unlock);
    Ring_Buffer_Init(&uart1_rx_rb, uart_rx_mem, UART_RX_RINGBUFFER_SIZE, ringbuffer_lock, ringbuffer_unlock);
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
    .bits.TransferSize = 4095
};
static dma_lli_ctrl_t uart_lli_list = 
{
    .src_addr = (uint32_t)src_buffer,
    .dst_addr = DMA_ADDR_UART1_TDR,
    .nextlli = 0
};

extern void led_toggle(uint8_t idx);
void uart_send_from_ringbuffer(void)
{
    if(Ring_Buffer_Get_Length(&usb_rx_rb))
    {
        if (!device_control(dma_ch2, DMA_CHANNEL_GET_STATUS, NULL))
        {
            uint32_t avalibleCnt = Ring_Buffer_Read(&usb_rx_rb, src_buffer, UART_TX_DMA_SIZE);
            
            if (avalibleCnt)
            {
                dma_channel_stop(dma_ch2);
                uart_dma_ctrl_cfg.bits.TransferSize = avalibleCnt;
                memcpy(&uart_lli_list.cfg, &uart_dma_ctrl_cfg, sizeof(dma_control_data_t));
                device_control(dma_ch2,DMA_CHANNEL_UPDATE,(void*)((uint32_t)&uart_lli_list));
                dma_channel_start(dma_ch2);
            }
        }
    }
}

