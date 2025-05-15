#ifndef _IIC1_H_
#define _IIC1_H_
#include <stdint.h>
// iic.h
extern void (*delay_us)(uint32_t us);
extern void (*delay_ms)(uint32_t ms);

typedef enum
{
    I2C_STATE_IDLE = 0,
    I2C_STATE_BUSY,
    I2C_STATE_ERROR,
} i2c_state_t;

typedef enum
{
    iic_low,
    iic_hight = 1
} i2c_level;

typedef struct
{
    void (*iic_set_scl)(i2c_level leve);
    void (*iic_set_sda)(i2c_level leve);
    i2c_level (*iic_read_sda)(void);
} iic_dev;

int iic_start(iic_dev *dev);
int iic_stop(iic_dev *dev);
int iic_send_byte(iic_dev *dev, uint8_t data);
int iic_read_byte(iic_dev *dev, uint8_t *data, int ack);

void iic_delay_init(void (*us_func)(uint32_t), void (*ms_func)(uint32_t));
void iic_init(void);

void iic_send_ack(iic_dev *dev, i2c_level ack_level);
uint8_t iic_receive_ack(iic_dev *dev);

#endif  // !_IIC_H
