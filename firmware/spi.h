#ifndef _PERIPHERY_SPI_H
#define _PERIPHERY_SPI_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#define PG12 204   //DI 检测LoRa DIO0发送或者接收状态
#define PG11 203   //DO 复位

typedef enum 
{
	PIO_MODE_OUT = 0,
	PIO_MODE_IN,
} gpio_mode_t;


enum spi_error_code {
    SPI_ERROR_ARG           = -1, /* Invalid arguments */
    SPI_ERROR_OPEN          = -2, /* Opening SPI device */
    SPI_ERROR_QUERY         = -3, /* Querying SPI device settings */
    SPI_ERROR_CONFIGURE     = -4, /* Configuring SPI device */
    SPI_ERROR_TRANSFER      = -5, /* SPI transfer */
    SPI_ERROR_CLOSE         = -6, /* Closing SPI device */
};

typedef struct spi_handle {
    int fd;

    struct {
        int c_errno;
        char errmsg[96];
    } error;
} spi_t;

typedef enum spi_bit_order {
    MSB_FIRST,
    LSB_FIRST,
} spi_bit_order_t;

/* Primary Functions */
int spi_open(spi_t *spi, const char *path, unsigned int mode,
                uint32_t max_speed);
int spi_open_advanced(spi_t *spi, const char *path, unsigned int mode,
                        uint32_t max_speed, spi_bit_order_t bit_order,
                        uint8_t bits_per_word, uint8_t extra_flags);
int spi_transfer(spi_t *spi, const uint8_t *txbuf, uint8_t *rxbuf, size_t len);

int spi_transfer_readRigister(int fd, const uint8_t *txbuf, uint8_t *rxbuf, size_t len);

void setspihandle(spi_t *spi);

int spi_write(spi_t *spi, const uint8_t *txbuf, size_t len );

int spi_read(spi_t *spi, uint8_t *rxbuf, size_t len );

//spi_t *spi getspihandle();

int spi_close(spi_t *spi);

/* Getters */
int spi_get_mode(spi_t *spi, unsigned int *mode);
int spi_get_max_speed(spi_t *spi, uint32_t *max_speed);
int spi_get_bit_order(spi_t *spi, spi_bit_order_t *bit_order);
int spi_get_bits_per_word(spi_t *spi, uint8_t *bits_per_word);
int spi_get_extra_flags(spi_t *spi, uint8_t *extra_flags);

/* Setters */
int spi_set_mode(spi_t *spi, unsigned int mode);
int spi_set_max_speed(spi_t *spi, uint32_t max_speed);
int spi_set_bit_order(spi_t *spi, spi_bit_order_t bit_order);
int spi_set_bits_per_word(spi_t *spi, uint8_t bits_per_word);
int spi_set_extra_flags(spi_t *spi, uint8_t extra_flags);

/* Miscellaneous */
//int spi_fd(spi_t *spi);
int spi_fd();
spi_t * spi_handle();
int spi_tostring(spi_t *spi, char *str, size_t len);

/* Error Handling */
int spi_errno(spi_t *spi);
const char *spi_errmsg(spi_t *spi);

int InitExDIState(int GPIO);
int set_gpio_cfg(int gpio, gpio_mode_t attr);
//int GetEnIOInValue(int DI,int *state);
//int SetEnIOOutState(int GPOI_OUT, int state);


#ifdef __cplusplus
}
#endif

#endif

