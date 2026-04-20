#ifndef RTOS_ARM_UART_H
#define RTOS_ARM_UART_H

void uart_init(void);
void uart_putc(char c);
void uart_puts(const char *s);

/* Polled, non-blocking receive: returns the next byte (0-255) if one is
 * waiting in USART1's data register, or -1 if none is available yet.
 * Added for the ros2_demo bidirectional path (Phase 5) -- the original
 * Phase 8 demo only ever needed uart_putc()/uart_puts(). */
int uart_getc_nonblocking(void);

#endif
