#ifndef BSP_ADC_H
#define BSP_ADC_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ATmega328P 10-bit ADC, avr-libc register-level driver (replaces the
   Arduino `analogRead()` abstraction). Reference: AVCC (5 V) with external
   capacitor on AREF. The joystick in the original sketch reads analog pins
   A0..A3, which on the Uno map to ADC channels 0..3 (PC0..PC3). */

void adc_init(void);          /* power on ADC, AVCC ref, prescaler /128 */
uint16_t adc_read(uint8_t ch); /* blocking single conversion, returns 0..1023 */

#ifdef __cplusplus
}
#endif

#endif /* BSP_ADC_H */
