#ifndef CORE_ARM_CONTROL_H
#define CORE_ARM_CONTROL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Per-servo runtime model:
     MODE_HOLD : drive toward `target` (smooth ramp), then hold
     MODE_AUTO : joystick-like self-sweep (triangle wave) between min..max
   "STOP <id>" simply switches a servo back to HOLD (freezes current angle). */

typedef enum { MODE_HOLD = 0, MODE_AUTO = 1 } servo_mode_t;

void arm_init(void);                 /* reset all to 90 deg, MODE_HOLD */
void arm_tick(void);                 /* call periodically (~20 ms)    */

uint8_t arm_set_angle(uint8_t id, uint8_t angle); /* returns applied (clamped) angle, 255 if bad id */
void arm_nudge(uint8_t id, int8_t delta); /* step current angle by delta (clamped, no ramp) */
void arm_stop(uint8_t id);           /* freeze auto-sweep, hold angle          */
bool arm_auto(uint8_t id);           /* start self-sweep; false if bad id      */
void arm_reset(void);                /* all servos -> 90, MODE_HOLD            */

uint8_t arm_get_angle(uint8_t id);   /* 0 if bad id */
servo_mode_t arm_get_mode(uint8_t id);
bool arm_id_valid(uint8_t id);

void arm_status(void);               /* print "S6=.. S7=.. S8=.. S9=.." + modes */

#ifdef __cplusplus
}
#endif

#endif /* CORE_ARM_CONTROL_H */
