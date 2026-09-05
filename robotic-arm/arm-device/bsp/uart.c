#include "uart.h"

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/pgmspace.h>
#include <stdarg.h>
#include <string.h>
#include <stdio.h> /* vsnprintf_P */

/* ---- ring buffers -------------------------------------------------------- */
#define RX_BUF_SZ 64
#define TX_BUF_SZ 128

static volatile uint8_t rx_buf[RX_BUF_SZ];
static volatile uint8_t rx_head = 0; /* next write position (ISR) */
static volatile uint8_t rx_tail = 0; /* next read position (main)  */

static volatile uint8_t tx_buf[TX_BUF_SZ];
static volatile uint8_t tx_head = 0; /* next write (main)   */
static volatile uint8_t tx_tail = 0; /* next read (ISR)     */

/* emit a RAM string (used only by uart_printf for its local stack buffer) */
static void uart_puts_ram(const char *s) {
    if (!s) return;
    while (*s) uart_putc(*s++);
}

static uint8_t rx_full(void) {
    return ((rx_head + 1) % RX_BUF_SZ) == rx_tail;
}
static uint8_t tx_full(void) {
    return ((tx_head + 1) % TX_BUF_SZ) == tx_tail;
}

void uart_init(uint32_t baud) {
    /* reset heads/tails */
    rx_head = rx_tail = 0;
    tx_head = tx_tail = 0;

    uint16_t ubrr = (uint16_t)((F_CPU / (8UL * baud)) - 1UL); /* U2X0 = 1 */

    UCSR0A = (1 << U2X0);                 /* double speed for lower error   */
    UBRR0H = (uint8_t)(ubrr >> 8);
    UBRR0L = (uint8_t)(ubrr & 0xFF);

    UCSR0B = (1 << RXEN0) | (1 << TXEN0) | (1 << RXCIE0) | (1 << UDRIE0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00); /* 8N1 */

    /* flush any pending RX data */
    (void)UDR0;
}

ISR(USART_RX_vect) {
    uint8_t d = UDR0;
    if (!rx_full()) {
        rx_buf[rx_head] = d;
        rx_head = (rx_head + 1) % RX_BUF_SZ;
    }
}

ISR(USART_UDRE_vect) {
    if (tx_tail != tx_head) {
        UDR0 = tx_buf[tx_tail];
        tx_tail = (tx_tail + 1) % TX_BUF_SZ;
    } else {
        /* nothing left to send: disable UDRE interrupt */
        UCSR0B &= ~(1 << UDRIE0);
    }
}

void uart_putc(char c) {
    /* wait for room (ring is small; this is short) */
    while (tx_full()) { /* busy wait */ }
    tx_buf[tx_head] = (uint8_t)c;
    tx_head = (tx_head + 1) % TX_BUF_SZ;
    /* make sure the ISR will drain it */
    UCSR0B |= (1 << UDRIE0);
}

void uart_puts(PGM_P s) {
    if (!s) return;
    char c;
    while ((c = (char)pgm_read_byte(s++)) != '\0') uart_putc(c);
}

int uart_printf(PGM_P fmt, ...) {
    char buf[96];
    va_list ap;
    va_start(ap, fmt);
    int n = vsnprintf_P(buf, sizeof(buf), fmt, ap);
    va_end(ap);
    if (n > 0) uart_puts_ram(buf);
    return n;
}

int uart_getc_nowait(void) {
    if (rx_tail == rx_head) return -1;
    int c = rx_buf[rx_tail];
    rx_tail = (rx_tail + 1) % RX_BUF_SZ;
    return c;
}
