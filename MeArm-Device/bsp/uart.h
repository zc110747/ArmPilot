#ifndef BSP_UART_H
#define BSP_UART_H

#include <stdint.h>
#include <stddef.h>
#include <avr/pgmspace.h>

#ifdef __cplusplus
extern "C" {
#endif

/* USART0 @ 115200 8N1 (U2X double speed; ATmega328P / Arduino Uno,
   wired to USB-serial -> PC COM4). Keep in sync with uart_init() in main.cpp.
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

/* ---- 链路丢弃计数（修任务④-②）-----------------------------------------
   ★ 为什么必须有这个：历史上「gripper 偶发不执行」之所以难查，是因为
   RX/TX 环满时的丢弃**完全静默** —— 症状只表现为"偶发"，没有任何读数。
   现在两处丢弃都计数，`STATS` 命令可读出：
     rx_drop > 0  ⇒ 收到的字节被丢过 ⇒ 某条指令可能残缺 ⇒ 对应 ACK 不会来
     tx_drop > 0  ⇒ 发出的字符被丢过 ⇒ 回执可能缺字（极少见，仅爆发输出时）
   ⚠️ 计数是**饱和的 uint8**（255 封顶），只用于"有没有发生过"，
      不适合当精确计量。压测时读一次、清一次即可。
   ⚠️ 读的时候字节可能只被读走一部分（ISR 还在跑），这是**可接受的近似**：
      我们只关心"是否为 0"。                                                    */
uint8_t uart_rx_drop_count(void);
uint8_t uart_tx_drop_count(void);
void uart_clear_drop_counters(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_UART_H */
