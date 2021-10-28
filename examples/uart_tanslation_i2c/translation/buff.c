#include "buff.h"



#define likely(x) __builtin_expect(!!(x), 1) //gcc内置函数, 帮助编译器分支优化
#define unlikely(x) __builtin_expect(!!(x), 0)

#define BUFFER_MAX (8 * 1024) //缓冲区大小
// #define BUFFER_MAX (256) //缓冲区大小
#define io1_HIGH ((*(volatile uint32_t *)0x40000188) |= (1 << 1))
#define io1_LOW ((*(volatile uint32_t *)0x40000188) &= (~(1 << 1)))
typedef struct _circle_buffer
{
    uint32_t head_pos; //缓冲区头部位置
    uint32_t tail_pos; //缓冲区尾部位置
    uint32_t len;

    uint8_t circle_buffer[BUFFER_MAX]; //缓冲区数组
} circle_buffer;
circle_buffer buffer_A;
circle_buffer buffer_B;
uint8_t buff_r = 0;         //i2c发生读行为  未读为0 读了为1

uint8_t buf_flage = 0;

void buff_init(void)
{
    buffer_A.head_pos = 0;
    buffer_A.tail_pos = 0;
    buffer_A.len = 0;

    buffer_B.head_pos = 0;
    buffer_B.tail_pos = 0;
    buffer_B.len = 0;
}

uint8_t sw_flage = 0;
void buf_switch(int flag)
{
    if (flag == 0)
    {
        if (buf_flage == 0)
        {
            buf_flage = 1;
        }
        else if (buf_flage == 1)
        { //读A操作B
            if (buff_r == 0)           //当A没有被读时
            { //当A读完，B缓冲区写完后，符合条件进行切换
                buf_flage = 2;
                sw_flage = 0; //切换成功
                buf_clean(0);
            }
            else
            {
                sw_flage = 1; //切换未成功，下次入队时清0
            }
        }
        else
        { //读B操作A
            if ( buff_r == 0)       //当B没有被读时
            { //当B读完，A缓冲区写完后，符合条件进行切换
                buf_flage = 1;
                sw_flage = 0; //切换成功
                buf_clean(0);
            }
            else
            {
                sw_flage = 1; //切换未成功，下次入队时清0
            }
        }
    }
    else
    { //初始化A缓冲区
        buffer_A.head_pos = 0;
        buffer_A.tail_pos = 0;
        buffer_A.len = 0;
        buf_flage = 0;
    }
}

void buf_push(uint8_t data) //入队列
{
    if (buf_flage == 0 || buf_flage == 2) //透传 或者读B写A
    {
        if (sw_flage == 1) //入队时清0
        {
            buffer_A.head_pos = 0;
            buffer_A.tail_pos = 0;
            buffer_A.len = 0;
            sw_flage = 0;
        }
        if (buffer_A.len == BUFFER_MAX)
        {
            buffer_A.circle_buffer[buffer_A.tail_pos] = data;
            if(++ buffer_A.tail_pos == BUFFER_MAX) buffer_A.tail_pos = 0;
            if(++ buffer_A.head_pos == BUFFER_MAX) buffer_A.head_pos = 0;
        }
        else
        {
            buffer_A.circle_buffer[buffer_A.tail_pos] = data; //在尾部添加
            if(++ buffer_A.tail_pos == BUFFER_MAX) buffer_A.tail_pos = 0;
            buffer_A.len++;
        }
    }
    else
    // if(buf_flage == 1)      //    at A
    {                      //正在读A，写入B
        if (sw_flage == 1) //入队时清0
        {
            buffer_B.head_pos = 0;
            buffer_B.tail_pos = 0;
            buffer_B.len = 0;
            sw_flage = 0;
        }
        if (buffer_B.len == BUFFER_MAX)
        {
            buffer_B.circle_buffer[buffer_B.tail_pos] = data;
            if(++ buffer_B.tail_pos == BUFFER_MAX) buffer_B.tail_pos = 0;
            if(++ buffer_B.head_pos == BUFFER_MAX) buffer_B.head_pos = 0;
        }
        else
        {
            buffer_B.circle_buffer[buffer_B.tail_pos] = data; //在尾部添加
            if(++ buffer_B.tail_pos == BUFFER_MAX) buffer_B.tail_pos = 0;
            buffer_B.len++;
        }
    }
}
uint8_t buf_pop(void) //出队列
{
    uint8_t data;
    if (buf_flage == 0 || buf_flage == 1)
    {
        if (buffer_A.len == 0) //为空返回0
            data = 0x00;
        else
        {
            buff_r = 1;                                 //切换加锁
            data = buffer_A.circle_buffer[buffer_A.head_pos]; //如果缓冲区非空则取头节点值并偏移头节点
            if(++ buffer_A.head_pos == BUFFER_MAX) buffer_A.head_pos = 0;
            // buffer_A.head_pos++;                              //当buff溢出时，自动归0
            // if (++buffer_A.head_pos >= BUFFER_MAX)
            //     buffer_A.head_pos = 0;
            buffer_A.len--;
            if(buffer_A.len == 0)
            {
                buff_r = 0;                             //切换解锁
            }
        }
        return data;
    }
    else
    {
        if (buffer_B.len == 0) //如果头尾接触表示缓冲区为空
            data = 0x00;
        else
        {
            buff_r = 1;                                 //切换加锁
            data = buffer_B.circle_buffer[buffer_B.head_pos]; //如果缓冲区非空则取头节点值并偏移头节点
            if(++ buffer_B.head_pos == BUFFER_MAX) buffer_B.head_pos = 0 ;                              //当buff溢出时，自动归0
            // if (++buffer_B.head_pos >= BUFFER_MAX)
            //     buffer_B.head_pos = 0;
            buffer_B.len--;
            if(buffer_B.len == 0)
            {
                buff_r = 0;                             //切换解锁
            }
        }
        return data;
    }
}

void buf_clean(uint32_t num)
{
    if (buf_flage == 2 || buf_flage == 0)
    {
        if (num == 0 || num > buffer_A.len)
        {
            buffer_A.head_pos = 0;
            buffer_A.tail_pos = 0;
            buffer_A.len = 0;
        }
        else
        {
            buffer_A.len -= num;
            if(buffer_A.tail_pos >= num)
            {
                buffer_A.tail_pos -= num;
            }
            else
            {
                buffer_A.tail_pos = BUFFER_MAX - (num - buffer_A.tail_pos);
            }
            
        }
    }
    else
    // if (buf_flage == 1)
    {
        if (num == 0 || num > buffer_B.len)
        {
            buffer_B.head_pos = 0;
            buffer_B.tail_pos = 0;
            buffer_B.len = 0;
        }
        else
        {
            buffer_B.len -= num;
            if(buffer_B.tail_pos >= num)
            {
                buffer_B.tail_pos -= num;
            }
            else
            {
                buffer_B.tail_pos = BUFFER_MAX - (num - buffer_B.tail_pos);
            }
        }
    }
}
