#include "ir_seq.h"

#include <avr/pgmspace.h>
#include "bsp/systick.h"
#include "bsp/uart.h"
#include "core/arm_control.h"
#include "core/joystick.h"

/* ---- keyframe model --------------------------------------------------------
   A keyframe sets all four servo targets at once, then the engine waits until
   the move has *settled* (arm_all_reached) AND `hold_ms` has elapsed -- the
   hold gap guarantees the arm is physically at the pose before the next
   command (req 2). The 5 s mid pause is just a keyframe whose angles repeat
   the previous pose and whose hold_ms = SEQ_MID_PAUSE.                      */

typedef struct {
    uint8_t  b;      /* base  (servo 9) target angle */
    uint8_t  l;      /* left  (servo 8) target angle */
    uint8_t  r;      /* right (servo 7) target angle */
    uint8_t  g;      /* grip  (servo 6) target angle */
    uint16_t hold_ms;
} seq_kf_t;

#define SEQ_MID_PAUSE 5000u   /* 5 s hold inserted in the middle (req 1) */

/* All sequences are hand-authored safe poses inside the forced servo limits:
     base 30..150, left 20..100, right 80..160, grip 40..125.
   Each set = 3 keyframes + 5 s mid pause + 3 keyframes ~= 15 s before looping. */

static const seq_kf_t SEQ1[] PROGMEM = {   /* 1: grab & place */
    {90, 60, 110, 125, 1500},   /* reach, grip open */
    {90, 60, 110,  40, 1500},   /* close grip */
    {90, 40, 130,  40, 1500},   /* lift */
    {90, 40, 130,  40, SEQ_MID_PAUSE}, /* ---- 5 s mid hold ---- */
    {120,40, 130,  40, 1500},   /* rotate base */
    {120,80,  90,  40, 1500},   /* lower */
    {120,80,  90, 125, 1500},   /* release grip */
};

static const seq_kf_t SEQ3[] PROGMEM = {   /* 3: swing left-right */
    {30, 60, 110,  80, 1500},
    {90, 60, 110,  80, 1500},
    {150,60, 110,  80, 1500},
    {90, 60, 110,  80, SEQ_MID_PAUSE}, /* ---- 5 s mid hold ---- */
    {90, 40, 130,  80, 1500},
    {90, 80,  90,  80, 1500},
    {90, 60, 110,  80, 1500},
};

static const seq_kf_t SEQ7[] PROGMEM = {   /* 7: pitch up-down */
    {90, 20, 160,  80, 1500},   /* up   */
    {90, 40, 130,  80, 1500},
    {90, 60, 110,  80, 1500},
    {90, 60, 110,  80, SEQ_MID_PAUSE}, /* ---- 5 s mid hold ---- */
    {90, 80,  90,  80, 1500},   /* down */
    {90, 60, 110,  80, 1500},
    {90, 40, 130,  80, 1500},
};

static const seq_kf_t SEQ9[] PROGMEM = {   /* 9: open/close + rotate */
    {90, 60, 110,  40, 1500},   /* close */
    {90, 60, 110, 125, 1500},   /* open  */
    {90, 60, 110,  40, 1500},   /* close */
    {90, 60, 110,  80, SEQ_MID_PAUSE}, /* ---- 5 s mid hold ---- */
    {30, 60, 110,  80, 1500},   /* rotate left  */
    {150,60, 110,  80, 1500},   /* rotate right */
    {90, 60, 110,  80, 1500},   /* center */
};

/* index by button number (1,3,7,9); unused slots are NULL / 0 */
static const seq_kf_t *const SEQ_PTR[10] PROGMEM = {
    NULL, SEQ1, NULL, SEQ3, NULL, NULL, NULL, SEQ7, NULL, SEQ9
};
static const uint8_t SEQ_LEN[10] PROGMEM = {
    0, 7, 0, 7, 0, 0, 0, 7, 0, 7
};

typedef struct {
    uint8_t  which;     /* 1/3/7/9 */
    uint8_t  kf;        /* current keyframe index */
    uint32_t t_start;   /* systick when current keyframe entered */
    bool     running;
} seq_run_t;

static seq_run_t run;

void ir_seq_init(void) {
    run.running = false;
    run.which   = 0;
    run.kf      = 0;
    run.t_start = 0;
}

static uint8_t seq_len(uint8_t which) {
    return pgm_read_byte(&SEQ_LEN[which]);
}

static void apply_kf(uint8_t which, uint8_t i) {
    const seq_kf_t *p = (const seq_kf_t *)pgm_read_ptr(&SEQ_PTR[which]);
    uint8_t b = pgm_read_byte(&p[i].b);
    uint8_t l = pgm_read_byte(&p[i].l);
    uint8_t r = pgm_read_byte(&p[i].r);
    uint8_t g = pgm_read_byte(&p[i].g);
    arm_set_angle(9, b);
    arm_set_angle(8, l);
    arm_set_angle(7, r);
    arm_set_angle(6, g);
}

static uint16_t kf_hold(uint8_t which, uint8_t i) {
    const seq_kf_t *p = (const seq_kf_t *)pgm_read_ptr(&SEQ_PTR[which]);
    return pgm_read_word(&p[i].hold_ms);
}

void ir_seq_trigger(uint8_t which) {
    if (which != 1 && which != 3 && which != 7 && which != 9) return;
    if (seq_len(which) == 0) return;
    run.which   = which;
    run.kf      = 0;
    run.running = true;
    apply_kf(which, 0);
    run.t_start = systick_ms();
    uart_printf(PSTR("OK IRSEQ %u start (%u steps)\r\n"), (unsigned)which,
                (unsigned)seq_len(which));
}

void ir_seq_stop(void) {
    if (!run.running) return;
    run.running = false;
    uart_printf(PSTR("OK IRSEQ %u stop\r\n"), (unsigned)run.which);
    run.which = 0;
}

bool ir_seq_is_running(void) { return run.running; }
uint8_t ir_seq_which(void)   { return run.running ? run.which : 0; }

void ir_seq_tick(void) {
    if (!run.running) {
        /* while idle, discard any stray joystick edge so a move made before
           the sequence started (e.g. noise on floating ADC pins) cannot leak
           in and stop the sequence the instant it begins */
        joystick_consume_input();
        return;
    }

    /* ---- stop conditions (req 3) ---- */
    if (joystick_consume_input()) {        /* hardware/serial joystick command */
        uart_puts(PSTR("IRSEQ stop: joystick\r\n"));
        ir_seq_stop();
        return;
    }

    uint32_t now = systick_ms();
    uint32_t el  = now - run.t_start;
    bool reached = arm_all_reached();
    uint16_t hold = kf_hold(run.which, run.kf);

    /* advance only after the move settled AND the hold gap elapsed (req 2) */
    if (reached && el >= (uint32_t)hold) {
        run.kf++;
        if (run.kf >= seq_len(run.which)) run.kf = 0;   /* loop (req 1) */
        apply_kf(run.which, run.kf);
        run.t_start = now;
    }
}
