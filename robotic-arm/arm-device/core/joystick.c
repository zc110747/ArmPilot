#include "joystick.h"

#include <avr/pgmspace.h>
#include "bsp/adc.h"
#include "core/arm_control.h"

/* ADC channel per axis (Arduino A0..A3 -> PC0..PC3) and the servo it drives.
   Matches the reference sketch: A0=base(9) A1=left(8) A2=grip(6) A3=right(7).
   Held in flash (PROGMEM) to keep the 2 KB RAM free on the 328P. */
static const uint8_t J_CH[4] PROGMEM = {0, 1, 2, 3};
static const uint8_t J_ID[4] PROGMEM = {9, 8, 6, 7};

static bool g_enabled = true;

void joystick_init(void) {
    adc_init();
    g_enabled = true;
}

void joystick_set_enabled(bool on) { g_enabled = on; }
bool joystick_is_enabled(void)     { return g_enabled; }

/* Per-axis step (+1 / -1 / 0), identical sense to handleJoystickControl():
     base (9) : raw>800 -> -1, raw<200 -> +1
     left (8) : raw<200 -> -1, raw>800 -> +1   (inverted)
     grip (6) : raw>800 -> -1, raw<200 -> +1
     right(7) : raw>800 -> -1, raw<200 -> +1                                  */
int8_t joystick_delta(uint8_t id, int raw) {
    if (raw < 0) raw = 0;
    if (raw > 1023) raw = 1023;
    if (id == 8)
        return (raw < 200) ? -1 : (raw > 800 ? (int8_t)+1 : 0);
    return (raw > 800) ? -1 : (raw < 200 ? (int8_t)+1 : 0);
}

/* Called from the main loop (like the original loop() calling
   handleJoystickControl() every iteration). A centred stick (200..800) yields
   delta 0 -> no movement; the forced servo range in arm_nudge() enforces the
   original 30..150 / 20..100 / 40..125 / 80..160 limits. */
void joystick_scan(void) {
    if (!g_enabled) return;
    for (uint8_t i = 0; i < 4; i++) {
        int raw = (int)adc_read(pgm_read_byte(&J_CH[i]));
        uint8_t id = pgm_read_byte(&J_ID[i]);
        int8_t d = joystick_delta(id, raw);
        if (d) arm_nudge(id, d);
    }
}
