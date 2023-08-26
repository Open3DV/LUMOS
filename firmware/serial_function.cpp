#include "serial_function.h"
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <stdio.h>
#include <unistd.h>
#include <termios.h>

int open_serial(const char* serial)
{
    int fd_uart;
    struct termios uart_conf;

    fd_uart = open(serial, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_uart < 0)
    {
        return -1;
    }
    tcgetattr(fd_uart, &uart_conf);
    cfsetospeed(&uart_conf, B9600);
    uart_conf.c_lflag &= ~(ECHO | ICANON | IEXTEN | ISIG);
    uart_conf.c_iflag &= ~(BRKINT | ICRNL | INPCK | ISTRIP | IXON);
    uart_conf.c_oflag &= ~(OPOST);
    uart_conf.c_cflag &= ~(CSIZE | PARENB);
    uart_conf.c_cflag |= CS8;
    uart_conf.c_cc[VMIN] = 1;
    uart_conf.c_cc[VTIME] = 0;
    if (tcsetattr(fd_uart, TCSANOW, &uart_conf) < 0)
    {
        return -1;
    }
    if (fcntl(fd_uart, F_SETFL, 0) < 0)
    {
        return -1;
    }
    return fd_uart;
}

int serial_write(int handle, char* buff, unsigned long long send_size)
{
    tcflush(handle, TCIFLUSH);
    int len = write(handle, buff, send_size);
    if (len < 0)
    {
        return -1;
    }
    return len;
}

int serial_read(int handle, char* buff, unsigned long long read_size)
{
    int len = read(handle, buff, read_size);
    if (len < 0)
    {
        return -1;
    }
    tcflush(handle, TCIFLUSH);
    return len;
}
