#include "iic.h"

#include "com_tool.h"

// iic.c
static void (*delay_us)(uint32_t us);
static void (*delay_ms)(uint32_t ms);

// 初始化函数：在程序初始化时调用一次
void iic_delay_init(void (*us_func)(uint32_t), void (*ms_func)(uint32_t))
{
    delay_us = us_func;
    delay_ms = ms_func;
}

void iic_init()
{
    iic_delay_init(Com_Delay_us, HAL_Delay);
}

int iic_start(iic_dev *dev)
{
    if (!dev || !delay_us) return -1;
    dev->iic_set_sda(iic_hight);
    dev->iic_set_scl(iic_hight);
    delay_us(5);
    dev->iic_set_sda(iic_low);
    delay_us(5);
    dev->iic_set_scl(iic_low);
    return 0;
}

int iic_stop(iic_dev *dev)
{
    if (!dev || !delay_us) return -1;
    dev->iic_set_scl(iic_low);
    dev->iic_set_sda(iic_low);
    delay_us(5);

    dev->iic_set_scl(iic_hight);
    delay_us(5);
    dev->iic_set_sda(iic_hight);
    delay_us(5);
    return 0;
}

int iic_send_byte(iic_dev *dev, uint8_t data)
{
    if (!dev || !delay_us) return -1;

    for (int i = 0; i < 8; i++)
    {
        dev->iic_set_sda((data & 0x80) ? iic_hight : iic_low);
        delay_us(2);
        dev->iic_set_scl(iic_hight);
        delay_us(5);
        dev->iic_set_scl(iic_low);
        delay_us(2);
        data <<= 1;
    }

    // 等待ACK
    dev->iic_set_sda(iic_hight);  // 释放 SDA
    delay_us(2);
    dev->iic_set_scl(iic_hight);
    delay_us(5);
    uint8_t ack = dev->iic_read_sda();
    dev->iic_set_scl(iic_low);
    return (ack == 0) ? 0 : -1;
}

int iic_read_byte(iic_dev *dev, uint8_t *data, int ack)
{
    if (!dev || !data || !delay_us) return -1;

    uint8_t val = 0;
    dev->iic_set_sda(iic_hight);  // release line
    for (int i = 0; i < 8; i++)
    {
        val <<= 1;
        dev->iic_set_scl(iic_hight);
        delay_us(5);
        if (dev->iic_read_sda()) val |= 0x01;
        dev->iic_set_scl(iic_low);
        delay_us(5);
    }
    *data = val;

	iic_send_ack(dev, (ack != 0) ? iic_low : iic_hight);
    return 0;
}

void iic_send_ack(iic_dev *dev, i2c_level ack_level)
{
    if (!dev || !delay_us) return;

    dev->iic_set_sda(ack_level);  // 设置 ACK 或 NACK（ACK=low, NACK=high）
    delay_us(2);
    dev->iic_set_scl(iic_hight);  // 拉高 SCL，产生时钟脉冲
    delay_us(5);
    dev->iic_set_scl(iic_low);  // 拉低 SCL，结束 ACK 位传输
    delay_us(2);
    dev->iic_set_sda(iic_hight);  // 释放 SDA，总线上为高电平
}

uint8_t iic_receive_ack(iic_dev *dev)
{
    if (!dev || !delay_us) return 1;  // 默认认为无ACK

    dev->iic_set_sda(iic_hight);  // 释放 SDA，准备读取 ACK
    delay_us(2);
    dev->iic_set_scl(iic_hight);  // 拉高 SCL，采样 SDA
    delay_us(5);
    uint8_t ack = dev->iic_read_sda();  // 读取 ACK 位（0 表示 ACK）
    dev->iic_set_scl(iic_low);          // 拉低 SCL，完成采样
    return ack;
}
