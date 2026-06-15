/*
 * serial.h - COM1 UART driver public API
 */
#pragma once

void serial_init(void);
void serial_putchar(char c);
void serial_puts(const char *s);
int  serial_getchar(void);    /* returns ASCII byte, or -1 if no data */
