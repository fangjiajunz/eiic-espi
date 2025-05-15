/**
 * @file spi1.c
 * @brief SPI协议实现文件
 */
#include "spi1.h"
#include "com_tool.h"

/* 私有变量 */
static void (*delay_us)(uint32_t us) = NULL;
static void (*delay_ms)(uint32_t ms) = NULL;
static uint32_t spi_timeout_ms = 1000;  // 默认超时时间，用于将来的超时功能实现

/**
 * @brief 获取SPI模式的时钟极性
 * 
 * @param mode SPI模式
 * @return spi_level_t 时钟极性(空闲状态)
 */
static inline spi_level_t spi_get_cpol(spi_mode_t mode) {
    return (mode == SPI_MODE2 || mode == SPI_MODE3) ? SPI_HIGH : SPI_LOW;
}

/**
 * @brief 初始化SPI延时函数
 */
spi_status_t spi_delay_init(void (*us_func)(uint32_t), void (*ms_func)(uint32_t)) {
    if (!us_func || !ms_func) {
        return SPI_ERROR_NULL_POINTER;
    }
    
    delay_us = us_func;
    delay_ms = ms_func;
    return SPI_OK;
}

/**
 * @brief 初始化SPI模块
 */
spi_status_t spi_init(void) {
    return spi_delay_init(Com_Delay_us, HAL_Delay);
}

/**
 * @brief 设置SPI超时时间
 * 
 * @param timeout_ms 超时时间(毫秒)
 */
void spi_set_timeout(uint32_t timeout_ms) {
    spi_timeout_ms = timeout_ms;
}

/**
 * @brief 初始化SPI设备
 */
spi_status_t spi_device_init(spi_dev_t *dev, const spi_config_t *config) {
    if (!dev || !config) {
        return SPI_ERROR_NULL_POINTER;
    }
    
    if (!delay_us || !delay_ms) {
        return SPI_ERROR_NULL_POINTER; // 确保延时函数已初始化
    }
    
    // 复制配置
    dev->config = *config;
    
    // 设置初始状态
    spi_level_t idle_level = spi_get_cpol(config->mode);
    dev->set_sck(idle_level);
    dev->set_mosi(SPI_LOW);
    
    if (config->cs_active_low) {
        dev->set_cs(SPI_HIGH); // 未选中状态
    } else {
        dev->set_cs(SPI_LOW);  // 未选中状态
    }
    
    dev->initialized = true;
    dev->is_busy = false;
    
    return SPI_OK;
}

/**
 * @brief 片选控制
 */
spi_status_t spi_chip_select(spi_dev_t *dev, bool select) {
    if (!dev || !dev->initialized) {
        return SPI_ERROR_NULL_POINTER;
    }
    
    if (select) {
        // 选中设备
        if (dev->config.cs_active_low) {
            dev->set_cs(SPI_LOW);
        } else {
            dev->set_cs(SPI_HIGH);
        }
    } else {
        // 释放设备
        if (dev->config.cs_active_low) {
            dev->set_cs(SPI_HIGH);
        } else {
            dev->set_cs(SPI_LOW);
        }
    }
    
    // 切换CS后等待稳定
    delay_us(1);
    
    return SPI_OK;
}

/**
 * @brief 发送一个字节数据
 */
spi_status_t spi_write_byte(spi_dev_t *dev, uint8_t data) {
    if (!dev || !dev->initialized || !delay_us) {
        return SPI_ERROR_NULL_POINTER;
    }
    
    if (dev->is_busy) {
        return SPI_ERROR_BUSY;
    }
    
    dev->is_busy = true;
    
    spi_mode_t mode = dev->config.mode;
    spi_level_t cpol = spi_get_cpol(mode);
    bool cpha = (mode == SPI_MODE1 || mode == SPI_MODE3);
    
    uint8_t bit_mask = (dev->config.bit_order == SPI_MSB_FIRST) ? 0x80 : 0x01;
    // 位移方向不再需要，直接在循环内处理
    
    // 计算适当的延时周期
    uint32_t half_cycle_us = (dev->config.clock_speed_hz > 0) 
                            ? (500000 / dev->config.clock_speed_hz) + 1  // 保证至少1us
                            : 1;  // 默认延时
    
    // 发送8位数据
    for (int i = 0; i < 8; i++) {
        // 准备数据位
        if (dev->config.bit_order == SPI_MSB_FIRST) {
            dev->set_mosi((data & bit_mask) ? SPI_HIGH : SPI_LOW);
            data <<= 1;
        } else {
            dev->set_mosi((data & bit_mask) ? SPI_HIGH : SPI_LOW);
            data >>= 1;
        }
        
        if (!cpha) {
            // CPHA=0: 在时钟第一个边沿采样
            delay_us(half_cycle_us);
            dev->set_sck(cpol ? SPI_LOW : SPI_HIGH);  // 切换时钟
            delay_us(half_cycle_us);
            dev->set_sck(cpol);   // 回到空闲状态
        } else {
            // CPHA=1: 在时钟第二个边沿采样
            dev->set_sck(cpol ? SPI_LOW : SPI_HIGH);  // 切换时钟
            delay_us(half_cycle_us);
            dev->set_sck(cpol);   // 回到空闲状态
            delay_us(half_cycle_us);
        }
    }
    
    dev->is_busy = false;
    return SPI_OK;
}

/**
 * @brief 读取一个字节数据
 */
spi_status_t spi_read_byte(spi_dev_t *dev, uint8_t *data) {
    if (!dev || !dev->initialized || !data || !delay_us) {
        return SPI_ERROR_NULL_POINTER;
    }
    
    if (dev->is_busy) {
        return SPI_ERROR_BUSY;
    }
    
    dev->is_busy = true;
    
    spi_mode_t mode = dev->config.mode;
    spi_level_t cpol = spi_get_cpol(mode);
    bool cpha = (mode == SPI_MODE1 || mode == SPI_MODE3);
    
    // 计算适当的延时周期
    uint32_t half_cycle_us = (dev->config.clock_speed_hz > 0) 
                            ? (500000 / dev->config.clock_speed_hz) + 1  // 保证至少1us
                            : 1;  // 默认延时
    
    uint8_t rx_data = 0;
    
    // MOSI拉低以避免干扰总线
    dev->set_mosi(SPI_LOW);
    
    // 接收8位数据
    for (int i = 0; i < 8; i++) {
        if (dev->config.bit_order == SPI_MSB_FIRST) {
            rx_data <<= 1;
        }
        
        if (!cpha) {
            // CPHA=0: 在时钟第一个边沿采样
            delay_us(half_cycle_us);
            dev->set_sck(cpol ? SPI_LOW : SPI_HIGH);  // 切换时钟
            
            // 采样数据
            if (dev->read_miso() == SPI_HIGH) {
                if (dev->config.bit_order == SPI_MSB_FIRST) {
                    rx_data |= 0x01;
                } else {
                    rx_data |= (0x80 >> i);
                }
            }
            
            delay_us(half_cycle_us);
            dev->set_sck(cpol);   // 回到空闲状态
        } else {
            // CPHA=1: 在时钟第二个边沿采样
            dev->set_sck(cpol ? SPI_LOW : SPI_HIGH);  // 切换时钟
            delay_us(half_cycle_us);
            dev->set_sck(cpol);   // 回到空闲状态
            
            // 采样数据
            if (dev->read_miso() == SPI_HIGH) {
                if (dev->config.bit_order == SPI_MSB_FIRST) {
                    rx_data |= 0x01;
                } else {
                    rx_data |= (0x80 >> i);
                }
            }
            
            delay_us(half_cycle_us);
        }
    }
    
    *data = rx_data;
    dev->is_busy = false;
    return SPI_OK;
}

/**
 * @brief 发送并接收一个字节数据
 */
spi_status_t spi_transfer_byte(spi_dev_t *dev, uint8_t data_out, uint8_t *data_in) {
    if (!dev || !dev->initialized || !data_in || !delay_us) {
        return SPI_ERROR_NULL_POINTER;
    }
    
    if (dev->is_busy) {
        return SPI_ERROR_BUSY;
    }
    
    dev->is_busy = true;
    
    spi_mode_t mode = dev->config.mode;
    spi_level_t cpol = spi_get_cpol(mode);
    bool cpha = (mode == SPI_MODE1 || mode == SPI_MODE3);
    
    uint8_t rx_data = 0;
    // 不再使用bit_mask变量，直接使用条件判断
    
    // 计算适当的延时周期
    uint32_t half_cycle_us = (dev->config.clock_speed_hz > 0) 
                            ? (500000 / dev->config.clock_speed_hz) + 1  // 保证至少1us
                            : 1;  // 默认延时
    
    // 发送和接收8位数据
    for (int i = 0; i < 8; i++) {
        // 准备发送数据位
        if (dev->config.bit_order == SPI_MSB_FIRST) {
            dev->set_mosi((data_out & 0x80) ? SPI_HIGH : SPI_LOW);
            data_out <<= 1;
            rx_data <<= 1;
        } else {
            dev->set_mosi((data_out & 0x01) ? SPI_HIGH : SPI_LOW);
            data_out >>= 1;
        }
        
        if (!cpha) {
            // CPHA=0: 在时钟第一个边沿采样
            delay_us(half_cycle_us);
            dev->set_sck(!cpol);  // 切换时钟
            
            // 采样数据
            if (dev->read_miso() == SPI_HIGH) {
                if (dev->config.bit_order == SPI_MSB_FIRST) {
                    rx_data |= 0x01;
                } else {
                    rx_data |= (0x80 >> i);
                }
            }
            
            delay_us(half_cycle_us);
            dev->set_sck(cpol);   // 回到空闲状态
        } else {
            // CPHA=1: 在时钟第二个边沿采样
            dev->set_sck(!cpol);  // 切换时钟
            delay_us(half_cycle_us);
            dev->set_sck(cpol);   // 回到空闲状态
            
            // 采样数据
            if (dev->read_miso() == SPI_HIGH) {
                if (dev->config.bit_order == SPI_MSB_FIRST) {
                    rx_data |= 0x01;
                } else {
                    rx_data |= (0x80 >> i);
                }
            }
            
            delay_us(half_cycle_us);
        }
    }
    
    *data_in = rx_data;
    dev->is_busy = false;
    return SPI_OK;
}

/**
 * @brief 发送多个字节
 */
spi_status_t spi_write_buffer(spi_dev_t *dev, const uint8_t *buffer, 
                              uint16_t length, bool cs_control) {
    if (!dev || !dev->initialized || !buffer) {
        return SPI_ERROR_NULL_POINTER;
    }
    
    // 控制CS
    if (cs_control) {
        spi_chip_select(dev, true);
    }
    
    // 循环发送每个字节
    spi_status_t status = SPI_OK;
    for (uint16_t i = 0; i < length; i++) {
        status = spi_write_byte(dev, buffer[i]);
        if (status != SPI_OK) {
            break;
        }
    }
    
    // 释放CS
    if (cs_control) {
        spi_chip_select(dev, false);
    }
    
    return status;
}

/**
 * @brief 读取多个字节
 */
spi_status_t spi_read_buffer(spi_dev_t *dev, uint8_t *buffer, 
                            uint16_t length, bool cs_control) {
    if (!dev || !dev->initialized || !buffer) {
        return SPI_ERROR_NULL_POINTER;
    }
    
    // 控制CS
    if (cs_control) {
        spi_chip_select(dev, true);
    }
    
    // 循环读取每个字节
    spi_status_t status = SPI_OK;
    for (uint16_t i = 0; i < length; i++) {
        status = spi_read_byte(dev, &buffer[i]);
        if (status != SPI_OK) {
            break;
        }
    }
    
    // 释放CS
    if (cs_control) {
        spi_chip_select(dev, false);
    }
    
    return status;
}

/**
 * @brief 发送并接收多个字节
 */
spi_status_t spi_transfer_buffer(spi_dev_t *dev, const uint8_t *tx_buffer, 
                                uint8_t *rx_buffer, uint16_t length, bool cs_control) {
    if (!dev || !dev->initialized || !rx_buffer) {
        return SPI_ERROR_NULL_POINTER;
    }
    
    // 控制CS
    if (cs_control) {
        spi_chip_select(dev, true);
    }
    
    // 循环传输每个字节
    spi_status_t status = SPI_OK;
    for (uint16_t i = 0; i < length; i++) {
        uint8_t tx_data = tx_buffer ? tx_buffer[i] : 0xFF;  // 如果没有发送数据，发送0xFF
        status = spi_transfer_byte(dev, tx_data, &rx_buffer[i]);
        if (status != SPI_OK) {
            break;
        }
    }
    
    // 释放CS
    if (cs_control) {
        spi_chip_select(dev, false);
    }
    
    return status;
}