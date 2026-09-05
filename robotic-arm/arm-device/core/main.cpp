extern "C" {
#include "bsp/uart.h"
#include "bsp/servo.h"
#include "bsp/led.h"
#include "core/arm_control.h"
#include "core/cmd.h"
#include "core/joystick.h"
#include "core/ir_ctrl.h"
}

#include <avr/interrupt.h>
#include <util/delay.h>

/* main loop runs at ~20 ms cadence (see _delay_ms below).
   25 iterations == 500 ms -> toggle the heartbeat LED once per 500 ms. */
#define LED_DIV  25

int main(void) {
    uart_init(9600);          /* COM4 @ 9600 8N1, bidirectional */
    servo_init();             /* Timer1 4-servo scheduler, all -> 90 */
    led_init();               /* onboard LED heartbeat, start ON */
    joystick_init();          /* ADC + hardware joystick scan (enabled) */
    ir_ctrl_init();           /* NEC IR receiver on PD2 (enabled) */
    arm_init();               /* app model reset to 90, MODE_HOLD */
    sei();                    /* enable global interrupts (servo/IR/timer) */

    uart_puts(PSTR("\r\n[meArm] bare-metal AVR ready, servos reset to 90\r\n"));
    uart_puts(PSTR("        type HELP for commands\r\n"));
    arm_status();

    uint8_t led_div = 0;
    for (;;) {
        cmd_poll();          /* process incoming serial commands */
        arm_tick();          /* ramp / auto-sweep (~20 ms cadence) */
        joystick_scan();     /* hardware joystick -> arm_nudge (if enabled) */
        ir_ctrl_poll();      /* hardware IR remote -> arm_nudge (if enabled) */
        if (++led_div >= LED_DIV) { led_div = 0; led_toggle(); }
        _delay_ms(20);
    }
    return 0;
}
