#include "arm_control.h"
#include "bsp/servo.h"
#include "bsp/uart.h"
#include <avr/pgmspace.h>

/* degrees advanced per arm_tick() (loop ~20 ms) -> limits slew speed */
#define RAMP_STEP 3

typedef struct {
    uint8_t      id;     /* user id 6/7/8/9 */
    servo_ch_t   ch;     /* hardware channel */
    uint8_t      target; /* desired angle (HOLD) */
    uint8_t      current;/* angle actually commanded this tick */
    servo_mode_t mode;
    int8_t       dir;    /* sweep direction for MODE_AUTO */
} arm_servo_t;

static arm_servo_t G[SERVO_COUNT];

static void sync_to_servo(arm_servo_t *s) {
    servo_set_angle(s->ch, s->current);
}

void arm_init(void) {
    /* build table in fixed id order: 9,8,7,6 */
    static const uint8_t ids[SERVO_COUNT] PROGMEM = {9, 8, 7, 6};
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        bool ok;
        uint8_t id = pgm_read_byte(&ids[i]);
        servo_ch_t c = servo_id_to_ch(id, &ok);
        G[i].id = id;
        G[i].ch = c;
        G[i].target = 90;
        G[i].current = 90;
        G[i].mode = MODE_HOLD;
        G[i].dir = 1;
        sync_to_servo(&G[i]);
    }
}

static arm_servo_t *find(uint8_t id) {
    for (uint8_t i = 0; i < SERVO_COUNT; i++)
        if (G[i].id == id) return &G[i];
    return NULL;
}

bool arm_id_valid(uint8_t id) {
    return find(id) != NULL;
}

void arm_tick(void) {
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        arm_servo_t *s = &G[i];
        if (s->mode == MODE_AUTO) {
            int next = (int)s->current + s->dir * RAMP_STEP;
            if (next >= servo_max_for(s->ch) ) { next = servo_max_for(s->ch); s->dir = -1; }
            else if (next <= servo_min_for(s->ch)) { next = servo_min_for(s->ch); s->dir = 1; }
            s->current = (uint8_t)next;
            s->target = s->current;
        } else { /* MODE_HOLD: ramp toward target */
            if (s->current < s->target) {
                s->current = (uint8_t)(s->current + RAMP_STEP);
                if (s->current > s->target) s->current = s->target;
            } else if (s->current > s->target) {
                s->current = (uint8_t)(s->current - RAMP_STEP);
                if (s->current < s->target) s->current = s->target;
            }
        }
        sync_to_servo(s);
    }
}

uint8_t arm_set_angle(uint8_t id, uint8_t angle) {
    arm_servo_t *s = find(id);
    if (!s) return 255;
    /* clamp to the servo's forced range */
    uint8_t lo = servo_min_for(s->ch);
    uint8_t hi = servo_max_for(s->ch);
    if (angle < lo) angle = lo;
    if (angle > hi) angle = hi;
    s->target = angle;
    s->mode = MODE_HOLD;
    return angle;
}

void arm_stop(uint8_t id) {
    arm_servo_t *s = find(id);
    if (!s) return;
    s->mode = MODE_HOLD;   /* freeze: hold current angle, cancel auto */
    s->target = s->current;
}

/* Joystick / IR style single-step nudge: move current angle by `delta`
   degrees, clamped to the servo's forced range, with no ramp. Both current
   and target are updated so arm_tick() leaves it where it is. */
void arm_nudge(uint8_t id, int8_t delta) {
    arm_servo_t *s = find(id);
    if (!s || delta == 0) return;
    int v = (int)s->current + delta;
    uint8_t lo = servo_min_for(s->ch);
    uint8_t hi = servo_max_for(s->ch);
    if (v < lo) v = lo;
    if (v > hi) v = hi;
    s->current = (uint8_t)v;
    s->target  = s->current;
    s->mode    = MODE_HOLD;
    sync_to_servo(s);
}

bool arm_auto(uint8_t id) {
    arm_servo_t *s = find(id);
    if (!s) return false;
    s->mode = MODE_AUTO;
    s->dir = (s->current <= (servo_min_for(s->ch) + servo_max_for(s->ch)) / 2) ? 1 : -1;
    return true;
}

void arm_reset(void) {
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        G[i].target = 90;
        G[i].current = 90;
        G[i].mode = MODE_HOLD;
        G[i].dir = 1;
        sync_to_servo(&G[i]);
    }
}

uint8_t arm_get_angle(uint8_t id) {
    arm_servo_t *s = find(id);
    return s ? s->current : 0;
}

servo_mode_t arm_get_mode(uint8_t id) {
    arm_servo_t *s = find(id);
    return s ? s->mode : MODE_HOLD;
}

bool arm_all_reached(void) {
    for (uint8_t i = 0; i < SERVO_COUNT; i++)
        if (G[i].current != G[i].target) return false;
    return true;
}

void arm_status(void) {
    char m6 = arm_get_mode(6) == MODE_AUTO ? 'A' : 'H';
    char m7 = arm_get_mode(7) == MODE_AUTO ? 'A' : 'H';
    char m8 = arm_get_mode(8) == MODE_AUTO ? 'A' : 'H';
    char m9 = arm_get_mode(9) == MODE_AUTO ? 'A' : 'H';
    uart_printf(PSTR("STATUS S6=%u(%c) S7=%u(%c) S8=%u(%c) S9=%u(%c)\r\n"),
        arm_get_angle(6), m6,
        arm_get_angle(7), m7,
        arm_get_angle(8), m8,
        arm_get_angle(9), m9);
}
