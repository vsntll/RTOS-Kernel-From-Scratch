#include "uart.h"

#include <stdint.h>

/* USART2 on PA2 (TX) / PA3 (RX), AF7 -- the usual pins for ST-Link's
 * virtual COM port on an STM32F4 Discovery board, and what Renode's
 * bundled stm32f4_discovery platform wires up. Minimal polled TX only;
 * this demo never needs to receive anything. */

#define RCC_BASE 0x40023800u
#define RCC_AHB1ENR (*(volatile uint32_t *)(RCC_BASE + 0x30u))
#define RCC_APB1ENR (*(volatile uint32_t *)(RCC_BASE + 0x40u))

#define GPIOA_BASE 0x40020000u
#define GPIOA_MODER (*(volatile uint32_t *)(GPIOA_BASE + 0x00u))
#define GPIOA_AFRL (*(volatile uint32_t *)(GPIOA_BASE + 0x20u))

#define USART2_BASE 0x40004400u
#define USART2_SR (*(volatile uint32_t *)(USART2_BASE + 0x00u))
#define USART2_DR (*(volatile uint32_t *)(USART2_BASE + 0x04u))
#define USART2_BRR (*(volatile uint32_t *)(USART2_BASE + 0x08u))
#define USART2_CR1 (*(volatile uint32_t *)(USART2_BASE + 0x0Cu))

#define USART_SR_TXE (1u << 7)
#define USART_CR1_TE (1u << 3)
#define USART_CR1_UE (1u << 13)

void uart_init(void) {
    RCC_AHB1ENR |= (1u << 0);  /* GPIOA clock */
    RCC_APB1ENR |= (1u << 17); /* USART2 clock */

    /* PA2/PA3 -> alternate function mode */
    GPIOA_MODER &= ~((3u << (2 * 2)) | (3u << (2 * 3)));
    GPIOA_MODER |= (2u << (2 * 2)) | (2u << (2 * 3));

    /* AF7 = USART2 on both pins */
    GPIOA_AFRL &= ~((0xFu << (4 * 2)) | (0xFu << (4 * 3)));
    GPIOA_AFRL |= (7u << (4 * 2)) | (7u << (4 * 3));

    USART2_BRR = 0x0683; /* ~9600 baud @ 16MHz APB1 -- not timing-critical here */
    USART2_CR1 = USART_CR1_UE | USART_CR1_TE;
}

void uart_putc(char c) {
    while (!(USART2_SR & USART_SR_TXE)) {
    }
    USART2_DR = (uint32_t)(unsigned char)c;
}

void uart_puts(const char *s) {
    while (*s) {
        uart_putc(*s++);
    }
}
