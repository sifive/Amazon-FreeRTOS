/* Copyright 2018 SiFive, Inc */
/* SPDX-License-Identifier: Apache-2.0 */

#include <metal/machine.h>
#include <metal/uart.h>

extern inline void metal_uart_init(struct metal_uart *uart, int baud_rate);
extern inline int metal_uart_putc(struct metal_uart *uart, unsigned char c);
extern inline int metal_uart_getc(struct metal_uart *uart, unsigned char *c);
extern inline int metal_uart_get_baud_rate(struct metal_uart *uart);
extern inline int metal_uart_set_baud_rate(struct metal_uart *uart, int baud_rate);

struct metal_uart *metal_uart_get_device(int device_num)
{
    if(device_num >= __METAL_DT_MAX_UARTS) {
        return NULL;
    }

    return &__metal_uart_table[device_num]->uart;
}
