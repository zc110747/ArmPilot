#ifndef BSP_SERVO_H
#define BSP_SERVO_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Mechanical-arm servos (ATmega328P). Pins mirror the original Arduino sketch:
     servo_9 (base rotation) -> D9  (PB1)
     servo_8 (left)          -> D8  (PB0)
     servo_7 (right)         -> D7  (PD7)
     servo_6 (grip)          -> D6  (PD6)
   Forced angle limits (degrees):
     base  : 30..150
     left  : 20..100
     right : 80..160
     grip  : 40..130
   On boot every servo is driven to 90 deg. */

typedef enum {
    SERVO_BASE  = 0, /* servo_9 */
    SERVO_LEFT  = 1, /* servo_8 */
    SERVO_RIGHT = 2, /* servo_7 */
    SERVO_GRIP  = 3, /* servo_6 */
    SERVO_COUNT = 4
} servo_ch_t;

/* user-facing servo id used on the serial interface (6/7/8/9) */
uint8_t servo_ch_to_id(servo_ch_t ch);
servo_ch_t servo_id_to_ch(uint8_t id, bool *ok);

void servo_init(void);
void servo_set_angle(servo_ch_t ch, uint8_t angle); /* clamped to [min,max] */
uint8_t servo_get_angle(servo_ch_t ch);
void servo_set_enabled(servo_ch_t ch, bool en);     /* false -> no pulse */
bool servo_is_enabled(servo_ch_t ch);

uint8_t servo_min_for(servo_ch_t ch); /* forced lower limit (deg) */
uint8_t servo_max_for(servo_ch_t ch); /* forced upper limit (deg) */

#ifdef __cplusplus
}
#endif

#endif /* BSP_SERVO_H */
