#include <stddef.h>
#include <stdint.h>
#include <stdarg.h>
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <unistd.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <linux/ioctl.h>
#include <linux/spi/spidev.h>
#include "spi.h"

spi_t *spihandle=NULL;

static void pabort(const char *s)
{
	perror(s);
	abort();
}

static int _spi_error(struct spi_handle *spi, int code, int c_errno, const char *fmt, ...) {
    va_list ap;

    spi->error.c_errno = c_errno;

    va_start(ap, fmt);
    vsnprintf(spi->error.errmsg, sizeof(spi->error.errmsg), fmt, ap);
    va_end(ap);

    /* Tack on strerror() and errno */
    if (c_errno) {
        char buf[64];
        strerror_r(c_errno, buf, sizeof(buf));
        snprintf(spi->error.errmsg+strlen(spi->error.errmsg), sizeof(spi->error.errmsg)-strlen(spi->error.errmsg), ": %s [errno %d]", buf, c_errno);
    }

    return code;
}

int spi_open(spi_t *spi, const char *path, unsigned int mode, uint32_t max_speed)
{
    return spi_open_advanced(spi, path, mode, max_speed, LSB_FIRST, 32, 0);
}


int spi_open2(spi_t *spi, const char *path, unsigned int mode, uint32_t max_speed)
{

	int ret=0;
	int bits=8;

	spi->fd = open(path, O_RDWR);
	if (spi->fd < 0)
		pabort("can't open device");

	/*
	 * spi mode
	 */
	ret = ioctl(spi->fd, SPI_IOC_WR_MODE, &mode);
	if (ret == -1)
		pabort("can't set spi mode");

	ret = ioctl(spi->fd, SPI_IOC_RD_MODE, &mode);
	if (ret == -1)
		pabort("can't get spi mode");
	

	/*
	 * bits per word
	 */
	ret = ioctl(spi->fd, SPI_IOC_WR_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't set bits per word");

	ret = ioctl(spi->fd, SPI_IOC_RD_BITS_PER_WORD, &bits);
	if (ret == -1)
		pabort("can't get bits per word");


	/*
	 * max speed hz
	 */
	ret = ioctl(spi->fd, SPI_IOC_WR_MAX_SPEED_HZ, &max_speed);
	if (ret == -1)
		pabort("can't set max speed hz");

	ret = ioctl(spi->fd, SPI_IOC_RD_MAX_SPEED_HZ, &max_speed);
	if (ret == -1)
		pabort("can't get max speed hz");
	
	return 0;
}


int spi_open_advanced(spi_t *spi, const char *path, unsigned int mode, uint32_t max_speed, spi_bit_order_t bit_order, uint8_t bits_per_word, uint8_t extra_flags) {
    uint8_t data8;

    /* Validate arguments */
    if (mode & ~0x3)
        return _spi_error(spi, SPI_ERROR_ARG, 0, "Invalid mode (can be 0,1,2,3)");
    if (bit_order != MSB_FIRST && bit_order != LSB_FIRST)
        return _spi_error(spi, SPI_ERROR_ARG, 0, "Invalid bit order (can be MSB_FIRST,LSB_FIRST)");

    memset(spi, 0, sizeof(struct spi_handle));

    /* Open device */
    if ((spi->fd = open(path, O_RDWR)) < 0)
        return _spi_error(spi, SPI_ERROR_OPEN, errno, "Opening SPI device \"%s\"", path);

    /* Set mode, bit order, extra flags */
    data8 = mode | ((bit_order == LSB_FIRST) ? SPI_LSB_FIRST : 0) | extra_flags;
    printf("mode value:%d\n",data8);
    if (ioctl(spi->fd, SPI_IOC_WR_MODE, &data8) < 0) {
        int errsv = errno;
        close(spi->fd);
        return _spi_error(spi, SPI_ERROR_CONFIGURE, errsv, "Setting SPI mode");
    }

    /* Set max speed */
    if (ioctl(spi->fd, SPI_IOC_WR_MAX_SPEED_HZ, &max_speed) < 0) {
        int errsv = errno;
        close(spi->fd);
        return _spi_error(spi, SPI_ERROR_CONFIGURE, errsv, "Setting SPI max speed");
    }

    /* Set bits per word */
    if (ioctl(spi->fd, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) < 0) {
        int errsv = errno;
        close(spi->fd);
        return _spi_error(spi, SPI_ERROR_CONFIGURE, errsv, "Setting SPI bits per word");
    }

    return 0;
}

int spi_transfer(spi_t *spi, const uint8_t *txbuf, uint8_t *rxbuf, size_t len)
{
    struct spi_ioc_transfer spi_xfer;

    /* Prepare SPI transfer structure */
    memset(&spi_xfer, 0, sizeof(struct spi_ioc_transfer));
    spi_xfer.tx_buf = (__u64)txbuf;
    spi_xfer.rx_buf = (__u64)rxbuf;
    spi_xfer.len = len;
    spi_xfer.delay_usecs = 0;
    spi_xfer.speed_hz = 0;
    spi_xfer.bits_per_word = 0;
    spi_xfer.cs_change = 0;

    /* Transfer */
    if (ioctl(spi->fd, SPI_IOC_MESSAGE(1), &spi_xfer) < 1)
        return _spi_error(spi, SPI_ERROR_TRANSFER, errno, "SPI transfer");

    return 0;
}

int spi_transfer_readRigister(int fd, const uint8_t *txbuf, uint8_t *rxbuf, size_t len)
{
    struct spi_ioc_transfer spi_xfer;

    /* Prepare SPI transfer structure */
    memset(&spi_xfer, 0, sizeof(struct spi_ioc_transfer));
    spi_xfer.tx_buf = (__u64)txbuf;
    spi_xfer.rx_buf = (__u64)rxbuf;
    spi_xfer.len = len;
    spi_xfer.delay_usecs = 0;
    spi_xfer.speed_hz = 0;
    spi_xfer.bits_per_word = 0;
    spi_xfer.cs_change = 0;

    /* Transfer */
    if (ioctl(fd, SPI_IOC_MESSAGE(1), &spi_xfer) < 1)
        //return _spi_error(spi, SPI_ERROR_TRANSFER, errno, "SPI transfer");

    return 0;
}


int spi_write(spi_t *spi, const uint8_t *txbuf, size_t len )
{
	int reback=0;

	reback=write(spi->fd,txbuf,len);
	if(reback < 0)
	{
		fprintf(stderr, "spi_write ():errno:%d --%s\n",errno,strerror(errno));
	}

	return reback;
}

int spi_read(spi_t *spi, uint8_t *rxbuf, size_t len )
{
	int reback=0;

    printf("spi read len: %ld\n", len);
	reback=read(spi->fd, rxbuf, len);
	if(reback < 0)
	{
		fprintf(stderr, "spi_read ():errno:%d --%s\n",errno,strerror(errno));
	}

	return reback;
}


void setspihandle(spi_t *spi)
{
    spihandle=spi;
    if(spihandle==NULL)
    {
        printf("set spi handle failed!\n");
    }
    else
    {
        printf("set spi handle is ok!\n");
    }
}

/*
spi_t *spi getspihandle()
{
    if(spihandle!=NULL)
    {
         return spihandle;
    }
    else
    {
        printf("the spihandle is NULL!\n");
        return NULL;
    }
}
*/


int spi_close(spi_t *spi) {
    if (spi->fd < 0)
        return 0;

    /* Close fd */
    if (close(spi->fd) < 0)
        return _spi_error(spi, SPI_ERROR_CLOSE, errno, "Closing SPI device");

    spi->fd = -1;

    return 0;
}

int spi_get_mode(spi_t *spi, unsigned int *mode) {
    uint8_t data8;

    if (ioctl(spi->fd, SPI_IOC_RD_MODE, &data8) < 0)
        return _spi_error(spi, SPI_ERROR_QUERY, errno, "Getting SPI mode");
    *mode = data8 & (SPI_CPHA | SPI_CPOL);

    return 0;
}

int spi_get_max_speed(spi_t *spi, uint32_t *max_speed) {
    uint32_t data32;

    if (ioctl(spi->fd, SPI_IOC_RD_MAX_SPEED_HZ, &data32) < 0)
        return _spi_error(spi, SPI_ERROR_QUERY, errno, "Getting SPI max speed");

    *max_speed = data32;

    return 0;
}

int spi_get_bit_order(spi_t *spi, spi_bit_order_t *bit_order) {
    uint8_t data8;

    if (ioctl(spi->fd, SPI_IOC_RD_LSB_FIRST, &data8) < 0)
        return _spi_error(spi, SPI_ERROR_QUERY, errno, "Getting SPI bit order");

    if (data8)
        *bit_order = LSB_FIRST;
    else
        *bit_order = MSB_FIRST;

    return 0;
}

int spi_get_bits_per_word(spi_t *spi, uint8_t *bits_per_word) {
    uint8_t data8;

    if (ioctl(spi->fd, SPI_IOC_RD_BITS_PER_WORD, &data8) < 0)
        return _spi_error(spi, SPI_ERROR_QUERY, errno, "Getting SPI bits per word");

    *bits_per_word = data8;

    return 0;
}

int spi_get_extra_flags(spi_t *spi, uint8_t *extra_flags) {
    uint8_t data8;

    if (ioctl(spi->fd, SPI_IOC_RD_MODE, &data8) < 0)
        return _spi_error(spi, SPI_ERROR_QUERY, errno, "Getting SPI mode flags");

    /* Extra mode flags without mode 0-3 and bit order */
    *extra_flags = data8 & ~( SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST );

    return 0;
}

int spi_set_mode(spi_t *spi, unsigned int mode) {
    uint8_t data8;

    if (mode & ~0x3)
        return _spi_error(spi, SPI_ERROR_ARG, 0, "Invalid mode (can be 0,1,2,3)");

    if (ioctl(spi->fd, SPI_IOC_RD_MODE, &data8) < 0)
        return _spi_error(spi, SPI_ERROR_QUERY, errno, "Getting SPI mode");

    data8 &= ~(SPI_CPOL | SPI_CPHA);
    data8 |= mode;

    if (ioctl(spi->fd, SPI_IOC_WR_MODE, &data8) < 0)
        return _spi_error(spi, SPI_ERROR_CONFIGURE, errno, "Setting SPI mode");

    return 0;
}

int spi_set_bit_order(spi_t *spi, spi_bit_order_t bit_order) {
    uint8_t data8;

    if (bit_order != MSB_FIRST && bit_order != LSB_FIRST)
        return _spi_error(spi, SPI_ERROR_ARG, 0, "Invalid bit order (can be MSB_FIRST,LSB_FIRST)");

    if (bit_order == LSB_FIRST)
        data8 = 1;
    else
        data8 = 0;

    if (ioctl(spi->fd, SPI_IOC_WR_LSB_FIRST, &data8) < 0)
        return _spi_error(spi, SPI_ERROR_CONFIGURE, errno, "Setting SPI bit order");

    return 0;
}

int spi_set_extra_flags(spi_t *spi, uint8_t extra_flags) {
    uint8_t data8;

    if (ioctl(spi->fd, SPI_IOC_RD_MODE, &data8) < 0)
        return _spi_error(spi, SPI_ERROR_QUERY, errno, "Getting SPI mode flags");

    /* Keep mode 0-3 and bit order */
    data8 &= (SPI_CPOL | SPI_CPHA | SPI_LSB_FIRST);
    /* Set extra flags */
    data8 |= extra_flags;

    if (ioctl(spi->fd, SPI_IOC_WR_MODE, &data8) < 0)
        return _spi_error(spi, SPI_ERROR_CONFIGURE, errno, "Setting SPI mode flags");

    return 0;
}

int spi_set_max_speed(spi_t *spi, uint32_t max_speed) {

    if (ioctl(spi->fd, SPI_IOC_WR_MAX_SPEED_HZ, &max_speed) < 0)
        return _spi_error(spi, SPI_ERROR_CONFIGURE, errno, "Setting SPI max speed");

    return 0;
}

int spi_set_bits_per_word(spi_t *spi, uint8_t bits_per_word) {

    if (ioctl(spi->fd, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) < 0)
        return _spi_error(spi, SPI_ERROR_CONFIGURE, errno, "Setting SPI bits per word");

    return 0;
}

int spi_tostring(spi_t *spi, char *str, size_t len) {
    unsigned int mode;
    char mode_str[2];
    uint32_t max_speed;
    char max_speed_str[16];
    uint8_t bits_per_word;
    char bits_per_word_str[4];
    spi_bit_order_t bit_order;
    char bit_order_str[16];
    uint8_t extra_flags;
    char extra_flags_str[4];

    if (spi_get_mode(spi, &mode) < 0)
        strncpy(mode_str, "?", sizeof(mode_str));
    else
        snprintf(mode_str, sizeof(mode_str), "%d", mode);

    if (spi_get_max_speed(spi, &max_speed) < 0)
        strncpy(max_speed_str, "?", sizeof(max_speed_str));
    else
        snprintf(max_speed_str, sizeof(max_speed_str), "%u", max_speed);

    if (spi_get_bit_order(spi, &bit_order) < 0)
        strncpy(bit_order_str, "?", sizeof(bit_order_str));
    else
        strncpy(bit_order_str, (bit_order == LSB_FIRST) ? "LSB first" : "MSB first", sizeof(bit_order_str));

    if (spi_get_bits_per_word(spi, &bits_per_word) < 0)
        strncpy(bits_per_word_str, "?", sizeof(bits_per_word_str));
    else
        snprintf(bits_per_word_str, sizeof(bits_per_word_str), "%u", bits_per_word);

    if (spi_get_extra_flags(spi, &extra_flags) < 0)
        strncpy(extra_flags_str, "?", sizeof(extra_flags_str));
    else
        snprintf(extra_flags_str, sizeof(extra_flags_str), "%02x", extra_flags);

    return snprintf(str, len, "SPI (fd=%d, mode=%s, max_speed=%s, bit_order=%s, bits_per_word=%s, extra_flags=%s)", spi->fd, mode_str, max_speed_str, bit_order_str, bits_per_word_str, extra_flags_str);
}

const char *spi_errmsg(spi_t *spi) {
    return spi->error.errmsg;
}

int spi_errno(spi_t *spi) {
    return spi->error.c_errno;
}

int spi_fd()
{
    if(spihandle!=NULL)
        return spihandle->fd;
    else
        return -1;
}

spi_t * spi_handle()
{
	return spihandle;
}


//GPIO CONTROL
int set_gpio_cfg(int gpio, gpio_mode_t attr)
{

	char buffer[32];
	int fd=-1;
	char filename[128] = "/sys/class/gpio/export";

	if ((fd = open(filename, O_WRONLY | O_NDELAY, 0)) == 0) {
		printf("Error: Not support gpio devices.\n\r");
		return -3;
	}

	sprintf( buffer, "%d" , gpio);
	write( fd, buffer, strlen(buffer) );
	close(fd);

	sprintf(filename,"/sys/class/gpio/gpio%d/direction",gpio);
	if ((fd = open(filename, O_WRONLY | O_NDELAY, 0)) == 0) {
		printf("Error: Not support gpio devices.\n\r");
		return -3;
	}

	if(attr == PIO_MODE_OUT)
		strcpy( buffer, "out" );
	else
		strcpy( buffer, "in" );

	write( fd, buffer, strlen(buffer) );
	close(fd);

	return 0;
}


#if 0
int GetEnIOInValue(int DI,int *state)
{
    int fd=-1;
	int gpios=0;
    char buffer[32]={0};
    char filename[128] = "/sys/class/gpio/export";
    int val=0;

	sprintf(filename,"/sys/class/gpio/gpio%d/value",DI);
	if ((fd = open(filename, O_RDONLY, 0)) == 0)
	{
		printf("Error: Not support gpio devices.\n\r");
		return -3;
	}

	read(fd, buffer, 1 );
	buffer[1]='\0';
	//printf("%02x\n",buffer[0]);
	*state = atoi(buffer);
	//printf("val:%d\n",val);
	//*state = val > 0 ? 0 : 1;

    close(fd);

	return 0;
}
#endif


int InitExDIState(int GPIO)
{
	 return set_gpio_cfg(GPIO,PIO_MODE_IN);	 
}

#if 0
int SetEnIOOutState(int GPOI_OUT, int state)
{
    int fd=-1;
	int gpio=0;
    char buffer[32]={0};
    char filename[128] = {0};
    int val=0;

	sprintf(filename,"/sys/class/gpio/gpio%d/value",GPOI_OUT);
	
	if ((fd = open(filename, O_WRONLY, 0)) == 0)
	{
		printf("Error: Not support gpio devices.\n\r");
		return -3;
	}
	memset(buffer,0,sizeof(buffer));

	if(state==0)
	{
		buffer[0]='0';
		buffer[1]='\0';
		write(fd,buffer,1);
	}
	else if(state==1)
	{
		buffer[0]='1';
		buffer[1]='\0';
		write(fd,buffer,1);
	}
	
    close(fd);
	
	return 0;
}
#endif 
