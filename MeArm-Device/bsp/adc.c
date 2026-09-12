#include "adc.h"

#include <avr/io.h>

/* Single ADC, 10-bit, AVCC reference, prescaler /128.
   At 16 MHz / 128 = 125 kHz ADC clock (within the 50..200 kHz spec).
   A conversion takes 13 cycles ~= 104 us; we block on ADSC. */
void adc_init(void) {
    ADMUX  = (1 << REFS0);                       /* AVCC, right-adjusted */
    ADCSRA = (1 << ADEN)                         /* enable */
           | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0); /* prescaler 128 */
    (void)adc_read(0);                           /* dummy conversion (1st is slow) */
}

uint16_t adc_read(uint8_t ch) {
    ADMUX  = (ADMUX & 0xF0) | (ch & 0x0F);       /* select channel, keep ref */
    ADCSRA |= (1 << ADSC);                       /* start conversion */
    while (ADCSRA & (1 << ADSC))                 /* wait for completion */
        ;
    return ADC;                                  /* ADCL read first, then ADCH (by macro) */
}
