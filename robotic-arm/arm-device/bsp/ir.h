#ifndef BSP_IR_H
#define BSP_IR_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* NEC infrared receiver driver (bare-metal, avr-libc).
   Replicates the original sketch's `IRrecv irrecv_2(2)` which listened on
   Arduino pin 2 (PD2 = INT0) for a 38 kHz NEC remote. A TSOP38xxx-style
   demodulated output (idle HIGH, mark = LOW) is decoded into the same 32-bit
   `decodedRawData` value the Arduino IRremote library produced:
       raw = address<<24 | ~address<<16 | command<<8 | ~command
   so the original 8 button codes (0xF708FF00 ...) stay valid. */

void ir_init(void);              /* PD2 INT0 + Timer0, start listening */
bool ir_get_code(uint32_t *out); /* true if a fresh NEC code is available  */

#ifdef __cplusplus
}
#endif

#endif /* BSP_IR_H */
