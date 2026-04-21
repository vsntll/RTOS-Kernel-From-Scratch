#include "uart.h"

#include <stdint.h>

/* USART1 on PA9 (TX) / PA10 (RX), AF7. Phase 8 originally used USART2 (the
 * ST-Link virtual COM port pins on a real Discovery board); switched to
 * USART1 for the Phase 0 QEMU port (see rtos/arm/README.md) because QEMU's
 * `netduinoplus2` STM32F405 model wires its USART1 instance to the first
 * `-serial` chardev (serial_hd(0)) -- there's no equivalent for picking
 * USART2 without a second -serial argument. Minimal polled TX only; this
 * demo never needs to receive anything. */

#define RCC_BASE 0x40023800u
#define RCC_AHB1ENR (*(volatile uint32_t *)(RCC_BASE + 0x30u))
#define RCC_APB2ENR (*(volatile uint32_t *)(RCC_BASE + 0x44u))

#define GPIOA_BASE 0x40020000u
#define GPIOA_MODER (*(volatile uint32_t *)(GPIOA_BASE + 0x00u))
#define GPIOA_AFRH (*(volatile uint32_t *)(GPIOA_BASE + 0x24u))

#define USART1_BASE 0x40011000u
#define USART1_SR (*(volatile uint32_t *)(USART1_BASE + 0x00u))
#define USART1_DR (*(volatile uint32_t *)(USART1_BASE + 0x04u))
#define USART1_BRR (*(volatile uint32_t *)(USART1_BASE + 0x08u))
#define USART1_CR1 (*(volatile uint32_t *)(USART1_BASE + 0x0Cu))

#define USART_SR_RXNE (1u << 5)
#define USART_SR_TXE (1u << 7)
#define USART_CR1_RE (1u << 2)
#define USART_CR1_TE (1u << 3)
#define USART_CR1_UE (1u << 13)

void uart_init(void) {
    RCC_AHB1ENR |= (1u << 0);  /* GPIOA clock */
    RCC_APB2ENR |= (1u << 4);  /* USART1 clock */

    /* PA9/PA10 -> alternate function mode. Pins 8-15 live in AFRH, not
     * AFRL (which only covers pins 0-7) -- the USART2 version of this code
     * used AFRL/pins 2-3 and doesn't translate directly. */
    GPIOA_MODER &= ~((3u << (2 * 9)) | (3u << (2 * 10)));
    GPIOA_MODER |= (2u << (2 * 9)) | (2u << (2 * 10));

    /* AF7 = USART1 on both pins. AFRH bit position for pin N is
     * 4*(N-8). */
    GPIOA_AFRH &= ~((0xFu << (4 * (9 - 8))) | (0xFu << (4 * (10 - 8))));
    GPIOA_AFRH |= (7u << (4 * (9 - 8))) | (7u << (4 * (10 - 8)));

    /* Baud isn't timing-critical for this demo (neither emulator's chardev
     * backend enforces real bit timing), and USART1 is on APB2 rather than
     * APB1 -- this BRR value is carried over unverified pending an actual
     * QEMU run; a wrong BRR still transmits complete bytes to the chardev,
     * it just wouldn't matter on real silicon either way here. */
    USART1_BRR = 0x0683;
    USART1_CR1 = USART_CR1_UE | USART_CR1_TE | USART_CR1_RE;
}

void uart_putc(char c) {
    while (!(USART1_SR & USART_SR_TXE)) {
    }
    USART1_DR = (uint32_t)(unsigned char)c;
}

void uart_puts(const char *s) {
    while (*s) {
        uart_putc(*s++);
    }
}

int uart_getc_nonblocking(void) {
    if (USART1_SR & USART_SR_RXNE) {
        return (int)(USART1_DR & 0xFFu);
    }
    return -1;
}
