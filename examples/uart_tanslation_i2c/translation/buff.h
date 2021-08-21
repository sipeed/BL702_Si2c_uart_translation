#ifndef __BUFF_H__
#define __BUFF_H__
#include<stdint.h>
void init();


void buf_push(uint8_t data);    //入队列
uint8_t buf_pop();              //出队列


void buf_clean(uint32_t num);
void buf_switch(int flag);

#endif