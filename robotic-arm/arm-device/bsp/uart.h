#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>
#include <stddef.h>
#include <avr/pgmspace.h>

#ifdef __cplusplus
extern "C" {
#endif

/* USART0 @ 9600 8N1 (ATmega328P / Arduino Uno, wired to USB-serial -> PC COM4).
   RX uses interrupt ring buffer; TX uses interrupt ring buffer so printing
   never blocks the servo ISR / main loop.

   IMPORTANT (AVR RAM budget): every string literal MUST live in flash, e.g.
       uart_puts(PSTR("hello"));
       uart_printf(PSTR("v=%u\r\n"), v);
   avr-gcc copies plain "..." literals into .data (RAM) at boot; on the 328P's
   2 KB that overflowed the stack and reset the MCU under command load. The
   _P variants below read the format/literal straight from flash (LPM).        */

void uart_init(uint32_t baud);

/* blocking-until-buffered send (returns after byte is queued, not after wire) */
void uart_putc(char c);
void uart_puts(PGM_P s);                 /* s MUST point into flash (PSTR) */

/* printf-like; format string MUST be in flash (PSTR). Uses avr-libc vsnprintf_P:
   %s reads a RAM string, %S (uppercase) reads a flash string. */
int uart_printf(PGM_P fmt, ...);

/* non-blocking RX: returns byte (0..255) or -1 when the ring is empty */
int uart_getc_nowait(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_UART_H */
