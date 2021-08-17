#include"buff.h"

#define BUFFER_MAX 256 //缓冲区大小

typedef struct _circle_buffer
{
  uint8_t head_pos; //缓冲区头部位置
  uint8_t tail_pos; //缓冲区尾部位置
  int len;

  uint8_t circle_buffer[BUFFER_MAX]; //缓冲区数组
} circle_buffer;
circle_buffer buffer_A;
circle_buffer buffer_B;


uint8_t buf_flage = 0;



void init()
{
    buffer_A.head_pos = 0;
    buffer_A.tail_pos = 0;
    buffer_A.len = 0;

    buffer_B.head_pos = 0;
    buffer_B.tail_pos = 0;
    buffer_B.len = 0;
}


void buf_switch(int flag)
{
    if(flag == 0)
    {
        if(buf_flage == 0 )
        {
            buf_flage = 1;
        }
        else if(buf_flage == 1)
        {                      //读A操作B
            if(buffer_A.len == 0)
            {                   //当A读完，B缓冲区写完后，符合条件进行切换
                buf_flage = 2;
            }
            else
            {                    //覆盖重写
                buffer_B.head_pos = 0;
                buffer_B.tail_pos = 0;
                buffer_B.len = 0; 
            }
        }
        else
        {                   //读B操作A
            if(buffer_B.len == 0)
            {                       //当B读完，A缓冲区写完后，符合条件进行切换
                buf_flage = 1;              
            }
            else
            {                   //覆盖重写
                buffer_A.head_pos = 0;
                buffer_A.tail_pos = 0;
                buffer_A.len = 0;
            }
        }
    }
    else
    {                           //初始化A缓冲区
        buffer_A.head_pos = 0;
        buffer_A.tail_pos = 0;
        buffer_A.len = 0; 
        buf_flage = 0;
    }

}

void buf_push(uint8_t data)    //入队列
{
    if(buf_flage == 0 || buf_flage == 2)          //透传 或者读B写A
    {                    
        if(buffer_A.len == 256)
        {
            buffer_A.circle_buffer[buffer_A.tail_pos] = data;
            buffer_A.tail_pos ++;
            buffer_A.head_pos ++;
        }
        else
        {
            buffer_A.circle_buffer[buffer_A.tail_pos] = data;//在尾部添加
            buffer_A.tail_pos ++;
            buffer_A.len ++;
        }
    }
    else 
    // if(buf_flage == 1)      //    at A
    {                                       //正在读A，写入B
        if(buffer_B.len == 256)
        {
            buffer_B.circle_buffer[buffer_B.tail_pos] = data;
            buffer_B.tail_pos ++;
            buffer_B.head_pos ++;
        }
        else
        {
            buffer_B.circle_buffer[buffer_B.tail_pos] = data;//在尾部添加
            buffer_B.tail_pos ++;
            buffer_B.len ++;
        }
    }
}
uint8_t buf_pop()              //出队列
{
    uint8_t data;
    if(buf_flage == 0 || buf_flage == 1)
    {
        if (buffer_A.len == 0) //如果头尾接触表示缓冲区为空
            data = 0x00;
        else
        {
            data = buffer_A.circle_buffer[buffer_A.head_pos]; //如果缓冲区非空则取头节点值并偏移头节点
            buffer_A.head_pos ++;   //当buff溢出时，自动归0
            // if (++buffer_A.head_pos >= BUFFER_MAX)
            //     buffer_A.head_pos = 0;
            buffer_A.len--;
        }
        return data;
    }
    else
    {
        if (buffer_B.len == 0) //如果头尾接触表示缓冲区为空
            data = 0x00;
        else
        {
            data = buffer_B.circle_buffer[buffer_B.head_pos]; //如果缓冲区非空则取头节点值并偏移头节点
            buffer_B.head_pos ++;//当buff溢出时，自动归0
            // if (++buffer_B.head_pos >= BUFFER_MAX)
            //     buffer_B.head_pos = 0;
            buffer_B.len--;
        }
        return data;
    }
}

void buf_clean(uint8_t num)
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
            buffer_A.tail_pos -= num;
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
            buffer_B.tail_pos -= num;

        }
    }
}









