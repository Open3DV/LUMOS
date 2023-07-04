#pragma once
#ifndef SERIAL_FUNCTION_H
#define SERIAL_FUNCTION_H

int open_serial(const char* serial);

int serial_write(int handle, char* buff, unsigned long long send_size);

int serial_read(int handle, char* buff, unsigned long long read_size);

#endif
