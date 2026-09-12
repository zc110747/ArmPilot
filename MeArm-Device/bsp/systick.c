#include "systick.h"

#include <avr/io.h>
#include <avr/interrupt.h>

static volatile uint32_t g_ms;

ISR(TIMER2_COMPA_vect) {
    g_ms++;
}

void systick_init(void) {
    /* Timer2 CTC mode, prescaler 64 (16 MHz / 64 = 250 kHz -> 4 us/tick).
       OCR2A = 249 -> 250 ticks * 4 us = 1 ms interrupt. */
    TCCR2A = (1 << WGM21);                 /* CTC, TOP = OCR2A */
    TCCR2B = (1 << CS22);                  /* prescaler 64 */
    OCR2A  = 249;
    TIMSK2 |= (1 << OCIE2A);               /* enable compare-A interrupt */
}

uint32_t systick_ms(void) {
    uint8_t sreg = SREG;
    cli();                                 /* 32-bit read must be atomic */
    uint32_t v = g_ms;
    SREG = sreg;
    return v;
}
