#include "led.h"

#include <avr/io.h>

/* Arduino Uno on-board LED is wired to PB5 (digital pin 13). */
#define LED_DDR  DDRB
#define LED_PORT PORTB
#define LED_BIT  PB5

void led_init(void) {
    LED_DDR |= (1u << LED_BIT);
    led_on();   /* start ON; main loop toggles every 500 ms */
}

void led_on(void)  { LED_PORT |=  (1u << LED_BIT); }
void led_off(void) { LED_PORT &= ~(1u << LED_BIT); }
void led_toggle(void) { LED_PORT ^= (1u << LED_BIT); }
