#include "servo.h"

#include <avr/io.h>
#include <avr/interrupt.h>

/* ---- hardware / timing constants ---------------------------------------- */
/* Timer1: CTC mode, TOP = ICR1, prescaler 8 (16MHz -> 2MHz -> 0.5us/tick).
   20 ms frame = 40000 ticks -> ICR1 = 39999.
   Servo pulse 1.0..2.0 ms maps to angle 0..180 deg. */
#define FRAME_TICKS 40000UL
#define ICR1_TOP    ((uint16_t)(FRAME_TICKS - 1))

/* angle (0..180) -> pulse width in ticks: 1000us..2000us -> 2000..4000 ticks */
static inline uint16_t angle_to_ticks(uint8_t angle) {
    /* ticks = 2000 + angle * (2000/180) */
    return (uint16_t)(2000UL + ((uint32_t)angle * 2000UL) / 180UL);
}

/* ---- per-channel descriptor --------------------------------------------- */
typedef struct {
    volatile uint8_t *port;
    volatile uint8_t *ddr;
    uint8_t bit;
    uint8_t ang_min;
    uint8_t ang_max;
} servo_hw_t;

static const servo_hw_t SERVO_HW[SERVO_COUNT] = {
    /* ch, port,        ddr,         bit,  min,  max */
    [SERVO_BASE]  = {&PORTB, &DDRB, PB1, 30, 150},
    [SERVO_LEFT]  = {&PORTB, &DDRB, PB0, 20, 100},
    [SERVO_RIGHT] = {&PORTD, &DDRD, PD7, 80, 160},
    [SERVO_GRIP]  = {&PORTD, &DDRD, PD6, 40, 125},
};

/* ---- runtime state ------------------------------------------------------ */
static volatile uint16_t g_pulse[SERVO_COUNT]; /* pulse ticks, 0 = disabled */
static volatile uint8_t  g_angle[SERVO_COUNT]; /* last commanded angle     */
static volatile bool     g_enabled[SERVO_COUNT];

/* frame scheduler (recomputed at frame start) */
static uint16_t s_event[SERVO_COUNT]; /* cumulative tick of each servo's falling edge */
static uint16_t s_pulse[SERVO_COUNT]; /* pulse ticks used this frame */

/* set/clear a servo output pin */
#define SET_PIN(i) (*SERVO_HW[i].port |=  (1u << SERVO_HW[i].bit))
#define CLR_PIN(i) (*SERVO_HW[i].port &= ~(1u << SERVO_HW[i].bit))

ISR(TIMER1_COMPA_vect) {
    static uint8_t idx = 0;

    if (idx == 0) {
        /* ---- frame start (timer just wrapped at TOP -> OCR1A == ICR1) ---- */
        uint16_t cum = 0;
        for (uint8_t i = 0; i < SERVO_COUNT; i++) {
            s_pulse[i] = g_enabled[i] ? g_pulse[i] : 0;
            cum += s_pulse[i];
            s_event[i] = cum;
        }
        if (s_pulse[0]) SET_PIN(0);
        OCR1A = s_event[0]; /* 0..16000, always < ICR1_TOP */
        idx = 1;
    } else if (idx < SERVO_COUNT) {
        /* falling edge of previous servo, rising edge of current */
        if (s_pulse[idx - 1]) CLR_PIN(idx - 1);
        if (s_pulse[idx]) SET_PIN(idx);
        OCR1A = s_event[idx];
        idx++;
    } else { /* idx == SERVO_COUNT: all pulses done, wait out the frame gap */
        if (s_pulse[SERVO_COUNT - 1]) CLR_PIN(SERVO_COUNT - 1);
        OCR1A = ICR1_TOP; /* match at TOP -> frame restart (idx stays, next = 0) */
        idx = 0;
    }
}

uint8_t servo_ch_to_id(servo_ch_t ch) {
    switch (ch) {
        case SERVO_BASE:  return 9;
        case SERVO_LEFT:  return 8;
        case SERVO_RIGHT: return 7;
        case SERVO_GRIP:  return 6;
        default:          return 0;
    }
}

servo_ch_t servo_id_to_ch(uint8_t id, bool *ok) {
    *ok = true;
    switch (id) {
        case 9: return SERVO_BASE;
        case 8: return SERVO_LEFT;
        case 7: return SERVO_RIGHT;
        case 6: return SERVO_GRIP;
        default: *ok = false; return SERVO_BASE;
    }
}

void servo_init(void) {
    /* configure pins as outputs, idle low */
    for (uint8_t i = 0; i < SERVO_COUNT; i++) {
        *SERVO_HW[i].ddr |= (1u << SERVO_HW[i].bit);
        CLR_PIN(i);
        g_angle[i] = 90;
        g_pulse[i] = angle_to_ticks(90);
        g_enabled[i] = true;
    }

    /* Timer1: CTC, TOP = ICR1, prescaler 8 */
    TCCR1A = 0;
    TCCR1B = (1 << WGM12) | (1 << WGM13) | (1 << CS11);
    ICR1 = ICR1_TOP;
    OCR1A = ICR1_TOP;
    TIMSK1 |= (1 << OCIE1A);
    /* timer starts counting; sei() is called by the application */
}

void servo_set_angle(servo_ch_t ch, uint8_t angle) {
    if (ch >= SERVO_COUNT) return;
    if (angle < SERVO_HW[ch].ang_min) angle = SERVO_HW[ch].ang_min;
    if (angle > SERVO_HW[ch].ang_max) angle = SERVO_HW[ch].ang_max;
    g_angle[ch] = angle;
    uint16_t ticks = angle_to_ticks(angle);
    uint8_t sreg = SREG; cli();   /* protect 16-bit write vs servo ISR */
    g_enabled[ch] = true;
    g_pulse[ch] = ticks;
    SREG = sreg;
}

uint8_t servo_get_angle(servo_ch_t ch) {
    if (ch >= SERVO_COUNT) return 0;
    return g_angle[ch];
}

void servo_set_enabled(servo_ch_t ch, bool en) {
    if (ch >= SERVO_COUNT) return;
    uint8_t sreg = SREG; cli();
    g_enabled[ch] = en;
    SREG = sreg;
    if (!en) CLR_PIN(ch); /* stop emitting a pulse immediately */
}

bool servo_is_enabled(servo_ch_t ch) {
    if (ch >= SERVO_COUNT) return false;
    return g_enabled[ch];
}

uint8_t servo_min_for(servo_ch_t ch) {
    if (ch >= SERVO_COUNT) return 0;
    return SERVO_HW[ch].ang_min;
}

uint8_t servo_max_for(servo_ch_t ch) {
    if (ch >= SERVO_COUNT) return 0;
    return SERVO_HW[ch].ang_max;
}
