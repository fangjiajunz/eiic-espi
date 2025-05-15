/**
 * @file spi1.h
 * @brief SPI协议接口头文件
 */
#ifndef SPI1_H_
#define SPI1_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief SPI信号电平定义
 */
typedef enum {
    SPI_LOW = 0,
    SPI_HIGH = 1
} spi_level_t;

/**
 * @brief SPI模式定义 (CPOL, CPHA组合)
 */
typedef enum {
    SPI_MODE0 = 0,  // CPOL=0, CPHA=0: 时钟空闲为低，第一个边沿采样
    SPI_MODE1 = 1,  // CPOL=0, CPHA=1: 时钟空闲为低，第二个边沿采样
    SPI_MODE2 = 2,  // CPOL=1, CPHA=0: 时钟空闲为高，第一个边沿采样
    SPI_MODE3 = 3   // CPOL=1, CPHA=1: 时钟空闲为高，第二个边沿采样
} spi_mode_t;

/**
 * @brief SPI位序定义
 */
typedef enum {
    SPI_MSB_FIRST = 0,  // 先发高位
    SPI_LSB_FIRST = 1   // 先发低位
} spi_bit_order_t;

/**
 * @brief SPI操作状态码
 */
typedef enum {
    SPI_OK = 0,
    SPI_ERROR_NULL_POINTER = -1,
    SPI_ERROR_TIMEOUT = -2,
    SPI_ERROR_BUSY = -3
} spi_status_t;

/**
 * @brief SPI设备配置结构体
 */
typedef struct {
    spi_mode_t mode;            // SPI模式
    spi_bit_order_t bit_order;  // 位序
    uint32_t clock_speed_hz;    // 时钟速度Hz
    bool cs_active_low;         // CS低电平有效标志
} spi_config_t;

/**
 * @brief SPI设备控制结构体
 */
typedef struct {
    // 引脚控制函数
    void (*set_sck)(spi_level_t level);
    void (*set_mosi)(spi_level_t level);
    void (*set_cs)(spi_level_t level);
    spi_level_t (*read_miso)(void);
    
    // 设备配置
    spi_config_t config;
    
    // 内部状态
    bool initialized;
    bool is_busy;
} spi_dev_t;

/**
 * @brief SPI传输结构体
 */
typedef struct {
    const uint8_t* tx_buffer;  // 发送缓冲区
    uint8_t* rx_buffer;        // 接收缓冲区
    uint16_t length;           // 传输长度
    uint16_t position;         // 当前位置
    bool cs_control;           // 是否控制CS
} spi_transfer_t;

/**
 * @brief 初始化SPI延时函数
 * 
 * @param us_func 微秒延时函数
 * @param ms_func 毫秒延时函数
 * @return spi_status_t 操作状态
 */
spi_status_t spi_delay_init(void (*us_func)(uint32_t), void (*ms_func)(uint32_t));

/**
 * @brief 初始化SPI模块
 * 
 * @return spi_status_t 操作状态
 */
spi_status_t spi_init(void);

/**
 * @brief 初始化SPI设备
 * 
 * @param dev SPI设备指针
 * @param config 设备配置
 * @return spi_status_t 操作状态
 */
spi_status_t spi_device_init(spi_dev_t *dev, const spi_config_t *config);

/**
 * @brief 片选控制
 * 
 * @param dev SPI设备指针
 * @param select true: 选中, false: 释放
 * @return spi_status_t 操作状态
 */
spi_status_t spi_chip_select(spi_dev_t *dev, bool select);

/**
 * @brief 发送一个字节数据
 * 
 * @param dev SPI设备指针
 * @param data 待发送字节
 * @return spi_status_t 操作状态
 */
spi_status_t spi_write_byte(spi_dev_t *dev, uint8_t data);

/**
 * @brief 读取一个字节数据
 * 
 * @param dev SPI设备指针
 * @param data 接收数据的指针
 * @return spi_status_t 操作状态
 */
spi_status_t spi_read_byte(spi_dev_t *dev, uint8_t *data);

/**
 * @brief 发送并接收一个字节数据
 * 
 * @param dev SPI设备指针
 * @param data_out 发送的字节
 * @param data_in 接收数据的指针
 * @return spi_status_t 操作状态
 */
spi_status_t spi_transfer_byte(spi_dev_t *dev, uint8_t data_out, uint8_t *data_in);

/**
 * @brief 发送多个字节
 * 
 * @param dev SPI设备指针
 * @param buffer 数据缓冲区
 * @param length 数据长度
 * @param cs_control 是否控制片选
 * @return spi_status_t 操作状态
 */
spi_status_t spi_write_buffer(spi_dev_t *dev, const uint8_t *buffer, 
                              uint16_t length, bool cs_control);

/**
 * @brief 读取多个字节
 * 
 * @param dev SPI设备指针
 * @param buffer 接收缓冲区
 * @param length 数据长度
 * @param cs_control 是否控制片选
 * @return spi_status_t 操作状态
 */
spi_status_t spi_read_buffer(spi_dev_t *dev, uint8_t *buffer, 
                             uint16_t length, bool cs_control);

/**
 * @brief 发送并接收多个字节
 * 
 * @param dev SPI设备指针
 * @param tx_buffer 发送缓冲区
 * @param rx_buffer 接收缓冲区
 * @param length 数据长度
 * @param cs_control 是否控制片选
 * @return spi_status_t 操作状态
 */
spi_status_t spi_transfer_buffer(spi_dev_t *dev, const uint8_t *tx_buffer, 
                                 uint8_t *rx_buffer, uint16_t length, bool cs_control);

#endif /* SPI1_H_ */
