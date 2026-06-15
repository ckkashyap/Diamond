/*
 * serial.c - COM1 UART 16550 driver (115200 8N1, polling)
 *
 * Also handles \n → \r\n expansion so callers can use plain \n.
 */

#include <stdint.h>
#include "../arch/x86/io.h"
#include "serial.h"

#define COM1 0x3F8u

/* 0 = no UART at COM1 (absent or didn't pass the scratch-register probe).
 * On hardware without a 16550 at 0x3F8 (e.g. Surface Laptop 3), the I/O
 * port floats 0xFF — without this flag, serial_getchar() would return 0xFF
 * on every call because LSR reads as 0xFF (Data-Ready always asserted).  */
static int s_com1_present = 0;

void serial_init(void) {
    /* Scratch-register probe: the 16550 Scratch reg (COM1+7) is a R/W byte
     * not used by the UART.  If we can write 0x55 / 0xAA and read them back,
     * a real UART is there.  A missing port floats 0xFF and fails the probe. */
    outb(COM1 + 7, 0x55);
    if (inb(COM1 + 7) != 0x55) return;
    outb(COM1 + 7, 0xAA);
    if (inb(COM1 + 7) != 0xAA) return;
    s_com1_present = 1;

    outb(COM1 + 1, 0x00);   /* disable all interrupts */
    outb(COM1 + 3, 0x80);   /* DLAB=1: access baud-rate divisor */
    outb(COM1 + 0, 0x01);   /* divisor LSB = 1 → 115200 baud */
    outb(COM1 + 1, 0x00);   /* divisor MSB = 0 */
    outb(COM1 + 3, 0x03);   /* 8 data, no parity, 1 stop; DLAB=0 */
    outb(COM1 + 2, 0xC7);   /* enable FIFO, clear, 14-byte threshold */
    outb(COM1 + 4, 0x0B);   /* DTR + RTS + OUT2 */
}

void serial_putchar(char c) {
    if (!s_com1_present) return;
    if (c == '\n') serial_putchar('\r');        /* expand LF → CR LF */
    while (!(inb(COM1 + 5) & 0x20));           /* wait: TX holding empty */
    outb(COM1, (uint8_t)c);
}

void serial_puts(const char *s) {
    for (; *s; s++) serial_putchar(*s);
}

int serial_getchar(void) {
    if (!s_com1_present) return -1;
    if (!(inb(COM1 + 5) & 0x01)) return -1;   /* no data available */
    return (int)(uint8_t)inb(COM1);
}
