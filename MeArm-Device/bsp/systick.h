#ifndef BSP_SYSTICK_H
#define BSP_SYSTICK_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Free-running 1 ms time base (Timer2, CTC, prescaler 64 -> 4 us/tick,
   OCR2A = 249 -> 250 * 4 us = 1 ms). Replaces the old _delay_ms(20) busy-wait
   in main(): the loop now spins freely and any timing (ramp cadence, LED
   heartbeat, IR sequence steps) is derived from systick_ms() instead of a
   blocking delay. */

void systick_init(void);
uint32_t systick_ms(void);   /* monotonic ms since boot (wraps at ~49 days) */

#ifdef __cplusplus
}
#endif

#endif /* BSP_SYSTICK_H */
