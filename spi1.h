/**
 * @file spi1.h
 * @brief SPIЭ��ӿ�ͷ�ļ�
 */
#ifndef SPI1_H_
#define SPI1_H_

#include <stdint.h>
#include <stdbool.h>

/**
 * @brief SPI�źŵ�ƽ����
 */
typedef enum {
    SPI_LOW = 0,
    SPI_HIGH = 1
} spi_level_t;

/**
 * @brief SPIģʽ���� (CPOL, CPHA���)
 */
typedef enum {
    SPI_MODE0 = 0,  // CPOL=0, CPHA=0: ʱ�ӿ���Ϊ�ͣ���һ�����ز���
    SPI_MODE1 = 1,  // CPOL=0, CPHA=1: ʱ�ӿ���Ϊ�ͣ��ڶ������ز���
    SPI_MODE2 = 2,  // CPOL=1, CPHA=0: ʱ�ӿ���Ϊ�ߣ���һ�����ز���
    SPI_MODE3 = 3   // CPOL=1, CPHA=1: ʱ�ӿ���Ϊ�ߣ��ڶ������ز���
} spi_mode_t;

/**
 * @brief SPIλ����
 */
typedef enum {
    SPI_MSB_FIRST = 0,  // �ȷ���λ
    SPI_LSB_FIRST = 1   // �ȷ���λ
} spi_bit_order_t;

/**
 * @brief SPI����״̬��
 */
typedef enum {
    SPI_OK = 0,
    SPI_ERROR_NULL_POINTER = -1,
    SPI_ERROR_TIMEOUT = -2,
    SPI_ERROR_BUSY = -3
} spi_status_t;

/**
 * @brief SPI�豸���ýṹ��
 */
typedef struct {
    spi_mode_t mode;            // SPIģʽ
    spi_bit_order_t bit_order;  // λ��
    uint32_t clock_speed_hz;    // ʱ���ٶ�Hz
    bool cs_active_low;         // CS�͵�ƽ��Ч��־
} spi_config_t;

/**
 * @brief SPI�豸���ƽṹ��
 */
typedef struct {
    // ���ſ��ƺ���
    void (*set_sck)(spi_level_t level);
    void (*set_mosi)(spi_level_t level);
    void (*set_cs)(spi_level_t level);
    spi_level_t (*read_miso)(void);
    
    // �豸����
    spi_config_t config;
    
    // �ڲ�״̬
    bool initialized;
    bool is_busy;
} spi_dev_t;

/**
 * @brief SPI����ṹ��
 */
typedef struct {
    const uint8_t* tx_buffer;  // ���ͻ�����
    uint8_t* rx_buffer;        // ���ջ�����
    uint16_t length;           // ���䳤��
    uint16_t position;         // ��ǰλ��
    bool cs_control;           // �Ƿ����CS
} spi_transfer_t;

/**
 * @brief ��ʼ��SPI��ʱ����
 * 
 * @param us_func ΢����ʱ����
 * @param ms_func ������ʱ����
 * @return spi_status_t ����״̬
 */
spi_status_t spi_delay_init(void (*us_func)(uint32_t), void (*ms_func)(uint32_t));

/**
 * @brief ��ʼ��SPIģ��
 * 
 * @return spi_status_t ����״̬
 */
spi_status_t spi_init(void);

/**
 * @brief ��ʼ��SPI�豸
 * 
 * @param dev SPI�豸ָ��
 * @param config �豸����
 * @return spi_status_t ����״̬
 */
spi_status_t spi_device_init(spi_dev_t *dev, const spi_config_t *config);

/**
 * @brief Ƭѡ����
 * 
 * @param dev SPI�豸ָ��
 * @param select true: ѡ��, false: �ͷ�
 * @return spi_status_t ����״̬
 */
spi_status_t spi_chip_select(spi_dev_t *dev, bool select);

/**
 * @brief ����һ���ֽ�����
 * 
 * @param dev SPI�豸ָ��
 * @param data �������ֽ�
 * @return spi_status_t ����״̬
 */
spi_status_t spi_write_byte(spi_dev_t *dev, uint8_t data);

/**
 * @brief ��ȡһ���ֽ�����
 * 
 * @param dev SPI�豸ָ��
 * @param data �������ݵ�ָ��
 * @return spi_status_t ����״̬
 */
spi_status_t spi_read_byte(spi_dev_t *dev, uint8_t *data);

/**
 * @brief ���Ͳ�����һ���ֽ�����
 * 
 * @param dev SPI�豸ָ��
 * @param data_out ���͵��ֽ�
 * @param data_in �������ݵ�ָ��
 * @return spi_status_t ����״̬
 */
spi_status_t spi_transfer_byte(spi_dev_t *dev, uint8_t data_out, uint8_t *data_in);

/**
 * @brief ���Ͷ���ֽ�
 * 
 * @param dev SPI�豸ָ��
 * @param buffer ���ݻ�����
 * @param length ���ݳ���
 * @param cs_control �Ƿ����Ƭѡ
 * @return spi_status_t ����״̬
 */
spi_status_t spi_write_buffer(spi_dev_t *dev, const uint8_t *buffer, 
                              uint16_t length, bool cs_control);

/**
 * @brief ��ȡ����ֽ�
 * 
 * @param dev SPI�豸ָ��
 * @param buffer ���ջ�����
 * @param length ���ݳ���
 * @param cs_control �Ƿ����Ƭѡ
 * @return spi_status_t ����״̬
 */
spi_status_t spi_read_buffer(spi_dev_t *dev, uint8_t *buffer, 
                             uint16_t length, bool cs_control);

/**
 * @brief ���Ͳ����ն���ֽ�
 * 
 * @param dev SPI�豸ָ��
 * @param tx_buffer ���ͻ�����
 * @param rx_buffer ���ջ�����
 * @param length ���ݳ���
 * @param cs_control �Ƿ����Ƭѡ
 * @return spi_status_t ����״̬
 */
spi_status_t spi_transfer_buffer(spi_dev_t *dev, const uint8_t *tx_buffer, 
                                 uint8_t *rx_buffer, uint16_t length, bool cs_control);

#endif /* SPI1_H_ */
