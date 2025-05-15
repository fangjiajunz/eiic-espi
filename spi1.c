/**
 * @file spi1.c
 * @brief SPIЭ��ʵ���ļ�
 */
#include "spi1.h"
#include "com_tool.h"

/* ˽�б��� */
static void (*delay_us)(uint32_t us) = NULL;
static void (*delay_ms)(uint32_t ms) = NULL;
static uint32_t spi_timeout_ms = 1000;  // Ĭ�ϳ�ʱʱ�䣬���ڽ����ĳ�ʱ����ʵ��

/**
 * @brief ��ȡSPIģʽ��ʱ�Ӽ���
 * 
 * @param mode SPIģʽ
 * @return spi_level_t ʱ�Ӽ���(����״̬)
 */
static inline spi_level_t spi_get_cpol(spi_mode_t mode) {
    return (mode == SPI_MODE2 || mode == SPI_MODE3) ? SPI_HIGH : SPI_LOW;
}

/**
 * @brief ��ʼ��SPI��ʱ����
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
 * @brief ��ʼ��SPIģ��
 */
spi_status_t spi_init(void) {
    return spi_delay_init(Com_Delay_us, HAL_Delay);
}

/**
 * @brief ����SPI��ʱʱ��
 * 
 * @param timeout_ms ��ʱʱ��(����)
 */
void spi_set_timeout(uint32_t timeout_ms) {
    spi_timeout_ms = timeout_ms;
}

/**
 * @brief ��ʼ��SPI�豸
 */
spi_status_t spi_device_init(spi_dev_t *dev, const spi_config_t *config) {
    if (!dev || !config) {
        return SPI_ERROR_NULL_POINTER;
    }
    
    if (!delay_us || !delay_ms) {
        return SPI_ERROR_NULL_POINTER; // ȷ����ʱ�����ѳ�ʼ��
    }
    
    // ��������
    dev->config = *config;
    
    // ���ó�ʼ״̬
    spi_level_t idle_level = spi_get_cpol(config->mode);
    dev->set_sck(idle_level);
    dev->set_mosi(SPI_LOW);
    
    if (config->cs_active_low) {
        dev->set_cs(SPI_HIGH); // δѡ��״̬
    } else {
        dev->set_cs(SPI_LOW);  // δѡ��״̬
    }
    
    dev->initialized = true;
    dev->is_busy = false;
    
    return SPI_OK;
}

/**
 * @brief Ƭѡ����
 */
spi_status_t spi_chip_select(spi_dev_t *dev, bool select) {
    if (!dev || !dev->initialized) {
        return SPI_ERROR_NULL_POINTER;
    }
    
    if (select) {
        // ѡ���豸
        if (dev->config.cs_active_low) {
            dev->set_cs(SPI_LOW);
        } else {
            dev->set_cs(SPI_HIGH);
        }
    } else {
        // �ͷ��豸
        if (dev->config.cs_active_low) {
            dev->set_cs(SPI_HIGH);
        } else {
            dev->set_cs(SPI_LOW);
        }
    }
    
    // �л�CS��ȴ��ȶ�
    delay_us(1);
    
    return SPI_OK;
}

/**
 * @brief ����һ���ֽ�����
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
    // λ�Ʒ�������Ҫ��ֱ����ѭ���ڴ���
    
    // �����ʵ�����ʱ����
    uint32_t half_cycle_us = (dev->config.clock_speed_hz > 0) 
                            ? (500000 / dev->config.clock_speed_hz) + 1  // ��֤����1us
                            : 1;  // Ĭ����ʱ
    
    // ����8λ����
    for (int i = 0; i < 8; i++) {
        // ׼������λ
        if (dev->config.bit_order == SPI_MSB_FIRST) {
            dev->set_mosi((data & bit_mask) ? SPI_HIGH : SPI_LOW);
            data <<= 1;
        } else {
            dev->set_mosi((data & bit_mask) ? SPI_HIGH : SPI_LOW);
            data >>= 1;
        }
        
        if (!cpha) {
            // CPHA=0: ��ʱ�ӵ�һ�����ز���
            delay_us(half_cycle_us);
            dev->set_sck(cpol ? SPI_LOW : SPI_HIGH);  // �л�ʱ��
            delay_us(half_cycle_us);
            dev->set_sck(cpol);   // �ص�����״̬
        } else {
            // CPHA=1: ��ʱ�ӵڶ������ز���
            dev->set_sck(cpol ? SPI_LOW : SPI_HIGH);  // �л�ʱ��
            delay_us(half_cycle_us);
            dev->set_sck(cpol);   // �ص�����״̬
            delay_us(half_cycle_us);
        }
    }
    
    dev->is_busy = false;
    return SPI_OK;
}

/**
 * @brief ��ȡһ���ֽ�����
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
    
    // �����ʵ�����ʱ����
    uint32_t half_cycle_us = (dev->config.clock_speed_hz > 0) 
                            ? (500000 / dev->config.clock_speed_hz) + 1  // ��֤����1us
                            : 1;  // Ĭ����ʱ
    
    uint8_t rx_data = 0;
    
    // MOSI�����Ա����������
    dev->set_mosi(SPI_LOW);
    
    // ����8λ����
    for (int i = 0; i < 8; i++) {
        if (dev->config.bit_order == SPI_MSB_FIRST) {
            rx_data <<= 1;
        }
        
        if (!cpha) {
            // CPHA=0: ��ʱ�ӵ�һ�����ز���
            delay_us(half_cycle_us);
            dev->set_sck(cpol ? SPI_LOW : SPI_HIGH);  // �л�ʱ��
            
            // ��������
            if (dev->read_miso() == SPI_HIGH) {
                if (dev->config.bit_order == SPI_MSB_FIRST) {
                    rx_data |= 0x01;
                } else {
                    rx_data |= (0x80 >> i);
                }
            }
            
            delay_us(half_cycle_us);
            dev->set_sck(cpol);   // �ص�����״̬
        } else {
            // CPHA=1: ��ʱ�ӵڶ������ز���
            dev->set_sck(cpol ? SPI_LOW : SPI_HIGH);  // �л�ʱ��
            delay_us(half_cycle_us);
            dev->set_sck(cpol);   // �ص�����״̬
            
            // ��������
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
 * @brief ���Ͳ�����һ���ֽ�����
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
    // ����ʹ��bit_mask������ֱ��ʹ�������ж�
    
    // �����ʵ�����ʱ����
    uint32_t half_cycle_us = (dev->config.clock_speed_hz > 0) 
                            ? (500000 / dev->config.clock_speed_hz) + 1  // ��֤����1us
                            : 1;  // Ĭ����ʱ
    
    // ���ͺͽ���8λ����
    for (int i = 0; i < 8; i++) {
        // ׼����������λ
        if (dev->config.bit_order == SPI_MSB_FIRST) {
            dev->set_mosi((data_out & 0x80) ? SPI_HIGH : SPI_LOW);
            data_out <<= 1;
            rx_data <<= 1;
        } else {
            dev->set_mosi((data_out & 0x01) ? SPI_HIGH : SPI_LOW);
            data_out >>= 1;
        }
        
        if (!cpha) {
            // CPHA=0: ��ʱ�ӵ�һ�����ز���
            delay_us(half_cycle_us);
            dev->set_sck(!cpol);  // �л�ʱ��
            
            // ��������
            if (dev->read_miso() == SPI_HIGH) {
                if (dev->config.bit_order == SPI_MSB_FIRST) {
                    rx_data |= 0x01;
                } else {
                    rx_data |= (0x80 >> i);
                }
            }
            
            delay_us(half_cycle_us);
            dev->set_sck(cpol);   // �ص�����״̬
        } else {
            // CPHA=1: ��ʱ�ӵڶ������ز���
            dev->set_sck(!cpol);  // �л�ʱ��
            delay_us(half_cycle_us);
            dev->set_sck(cpol);   // �ص�����״̬
            
            // ��������
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
 * @brief ���Ͷ���ֽ�
 */
spi_status_t spi_write_buffer(spi_dev_t *dev, const uint8_t *buffer, 
                              uint16_t length, bool cs_control) {
    if (!dev || !dev->initialized || !buffer) {
        return SPI_ERROR_NULL_POINTER;
    }
    
    // ����CS
    if (cs_control) {
        spi_chip_select(dev, true);
    }
    
    // ѭ������ÿ���ֽ�
    spi_status_t status = SPI_OK;
    for (uint16_t i = 0; i < length; i++) {
        status = spi_write_byte(dev, buffer[i]);
        if (status != SPI_OK) {
            break;
        }
    }
    
    // �ͷ�CS
    if (cs_control) {
        spi_chip_select(dev, false);
    }
    
    return status;
}

/**
 * @brief ��ȡ����ֽ�
 */
spi_status_t spi_read_buffer(spi_dev_t *dev, uint8_t *buffer, 
                            uint16_t length, bool cs_control) {
    if (!dev || !dev->initialized || !buffer) {
        return SPI_ERROR_NULL_POINTER;
    }
    
    // ����CS
    if (cs_control) {
        spi_chip_select(dev, true);
    }
    
    // ѭ����ȡÿ���ֽ�
    spi_status_t status = SPI_OK;
    for (uint16_t i = 0; i < length; i++) {
        status = spi_read_byte(dev, &buffer[i]);
        if (status != SPI_OK) {
            break;
        }
    }
    
    // �ͷ�CS
    if (cs_control) {
        spi_chip_select(dev, false);
    }
    
    return status;
}

/**
 * @brief ���Ͳ����ն���ֽ�
 */
spi_status_t spi_transfer_buffer(spi_dev_t *dev, const uint8_t *tx_buffer, 
                                uint8_t *rx_buffer, uint16_t length, bool cs_control) {
    if (!dev || !dev->initialized || !rx_buffer) {
        return SPI_ERROR_NULL_POINTER;
    }
    
    // ����CS
    if (cs_control) {
        spi_chip_select(dev, true);
    }
    
    // ѭ������ÿ���ֽ�
    spi_status_t status = SPI_OK;
    for (uint16_t i = 0; i < length; i++) {
        uint8_t tx_data = tx_buffer ? tx_buffer[i] : 0xFF;  // ���û�з������ݣ�����0xFF
        status = spi_transfer_byte(dev, tx_data, &rx_buffer[i]);
        if (status != SPI_OK) {
            break;
        }
    }
    
    // �ͷ�CS
    if (cs_control) {
        spi_chip_select(dev, false);
    }
    
    return status;
}