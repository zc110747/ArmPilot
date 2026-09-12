#ifndef CORE_JOYSTICK_H
#define CORE_JOYSTICK_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Hardware joystick support. Reads the 4 potentiometer axes through the ADC
   (original sketch used analogRead(A0..A3)) and applies the exact same
   thresholds / per-axis sense as `handleJoystickControl()` in the Arduino
   reference project. */

void joystick_init(void);          /* init ADC + enable scanning */
void joystick_scan(void);          /* read 4 axes, nudge servos (if enabled) */
int8_t joystick_delta(uint8_t id, int raw); /* axis mapping: per-call step */
void joystick_set_enabled(bool on);
bool joystick_is_enabled(void);

/* Returns true exactly once after a real joystick movement has been detected
   since the last call (any axis produced a non-zero step). The IR sequence
   engine polls this each loop so a joystick command ends the auto-loop (req 3).
   Consumed (reset) by the call, so it only fires on a fresh movement edge. */
bool joystick_consume_input(void);

#ifdef __cplusplus
}
#endif

#endif /* CORE_JOYSTICK_H */
