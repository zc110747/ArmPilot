#ifndef BSP_LED_H
#define BSP_LED_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Onboard status LED of the Arduino Uno: pin 13 = PB5.
   Used as a heartbeat to show the firmware is alive (default 500 ms blink). */

void led_init(void);   /* configure PB5 as output, start ON */
void led_on(void);
void led_off(void);
void led_toggle(void);

#ifdef __cplusplus
}
#endif

#endif /* BSP_LED_H */
