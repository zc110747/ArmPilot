#include "ir.h"

#include <avr/io.h>
#include <avr/interrupt.h>

/* ---- hardware / timing ----------------------------------------------------
   Arduino pin 2 = PD2 = INT0. The demodulated IR module output idles HIGH,
   mark (burst) = LOW. Timer0 runs free at prescaler 64 -> 4 us per tick
   (16 MHz / 64); its 8-bit overflow (~1.024 ms) is accumulated in ir_ovf to
   form a 16-bit time base (now, in ticks of 4 us).                         */
#define IR_PIN  PD2

#define T_START_MARK_LO 1600   /* 6.4  ms */
#define T_START_MARK_HI 2900   /* 11.6 ms */
#define T_START_SPC_LO   800   /* 3.2  ms */
#define T_START_SPC_HI  1500   /* 6.0  ms */
#define T_BIT1_THRESH    350   /* space > 1.4 ms -> logic 1 (else 0) */

static volatile uint8_t  ir_ovf;     /* Timer0 overflow count (time high byte) */
static volatile uint16_t ir_prev;    /* time of previous edge (ticks)         */
static volatile uint8_t  ir_state;   /* 0 idle, 1 got start mark, 2 receiving  */
static volatile uint32_t ir_data;    /* accumulating 32 NEC bits (LSB first)   */
static volatile uint8_t  ir_bits;
static volatile uint32_t ir_code;    /* latest decoded raw 32-bit code         */
static volatile uint8_t  ir_ready;   /* 1 = new code waiting                  */

ISR(TIMER0_OVF_vect) { ir_ovf++; }

/* current time base (ticks of 4 us), with a small atomicity guard */
static inline uint16_t ir_now(void) {
    uint8_t ovf1 = ir_ovf;
    uint8_t cnt  = TCNT0;
    uint8_t ovf2 = ir_ovf;
    if (ovf1 != ovf2) cnt = 0;        /* overflow raced the read; treat as wrap */
    return ((uint16_t)ovf2 << 8) | cnt;
}

ISR(INT0_vect) {
    uint16_t now = ir_now();
    uint8_t  lvl = (PIND & (1 << IR_PIN)) ? 1 : 0;
    uint16_t dt  = now - ir_prev;     /* ticks since previous edge */
    ir_prev = now;

    if (lvl == 1) {
        /* rising edge -> a mark just ended. Only the 9 ms start mark matters. */
        if (ir_state == 0) {
            if (dt >= T_START_MARK_LO && dt <= T_START_MARK_HI)
                ir_state = 1;
        }
        /* bit marks (562.5 us) are ignored while receiving */
    } else {
        /* falling edge -> a space just ended, next mark begins */
        if (ir_state == 1) {
            /* space after the 9 ms mark must be ~4.5 ms (start separator) */
            if (dt >= T_START_SPC_LO && dt <= T_START_SPC_HI) {
                ir_state = 2; ir_bits = 0; ir_data = 0;
            } else {
                ir_state = 0;          /* not a valid frame start */
            }
        } else if (ir_state == 2) {
            /* this space classifies the bit that just finished.
               NEC sends LSB first, so bit N goes into ir_data bit N. */
            uint8_t b = (dt > T_BIT1_THRESH) ? 1 : 0;
            ir_data |= ((uint32_t)b << ir_bits);
            ir_bits++;
            if (ir_bits >= 32) {
                uint8_t b0 = ir_data & 0xFF;
                uint8_t b1 = (ir_data >> 8)  & 0xFF;
                uint8_t b2 = (ir_data >> 16) & 0xFF;
                uint8_t b3 = (ir_data >> 24) & 0xFF;
                uint8_t inv0 = (uint8_t)~b0;
                uint8_t inv2 = (uint8_t)~b2;
                if (inv0 == b1 && inv2 == b3) {
                    ir_code = ((uint32_t)b0 << 24) | ((uint32_t)b1 << 16) |
                              ((uint32_t)b2 << 8)  | b3;
                    ir_ready = 1;
                }
                ir_state = 0;
            }
        }
    }
}

void ir_init(void) {
    /* Timer0: normal mode, prescaler 64 -> 4 us/tick, overflow interrupt */
    TCCR0A = 0;
    TCCR0B = (1 << CS01) | (1 << CS00);
    TIMSK0 |= (1 << TOIE0);

    /* IR input with pull-up (module idles HIGH; no connection stays HIGH) */
    DDRD  &= ~(1 << IR_PIN);
    PORTD |=  (1 << IR_PIN);

    /* INT0: trigger on any logical change */
    EICRA |= (1 << ISC00);
    EIMSK |= (1 << INT0);

    ir_ovf = 0; ir_state = 0; ir_bits = 0; ir_ready = 0; ir_prev = 0;
}

bool ir_get_code(uint32_t *out) {
    if (ir_ready) {
        *out = ir_code;
        ir_ready = 0;
        return true;
    }
    return false;
}

/* Discard any pending frame / partial state. Called when the hardware receiver
   is (re)enabled so a code that arrived while it was off is not replayed. */
void ir_flush(void) {
    ir_ready = 0;
    ir_state = 0;
    ir_bits  = 0;
}
