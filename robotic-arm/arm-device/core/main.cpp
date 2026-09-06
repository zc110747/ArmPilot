extern "C" {
#include "bsp/uart.h"
#include "bsp/servo.h"
#include "bsp/led.h"
#include "bsp/systick.h"
#include "core/arm_control.h"
#include "core/cmd.h"
#include "core/joystick.h"
#include "core/ir_ctrl.h"
#include "core/ir_seq.h"
}

#include <avr/interrupt.h>

/* The loop no longer busy-waits: it spins freely and derives all timing from
   the 1 ms systick (bsp/systick). Sub-systems that need a cadence check the
   tick themselves:
     - arm_tick()  : every 20 ms (servo ramp slew, same speed as before)
     - joystick    : every 20 ms (ADC scan)
     - ir_seq_tick : every loop iteration (reacts to stop/switch immediately)
     - LED heart   : toggles every 500 ms                                   */

#define SLOW_MS    30   /* arm_tick + joystick cadence */
#define LED_MS   500    /* heartbeat period */

int main(void) {
    uart_init(115200);        /* COM4 @ 115200 8N1, bidirectional */
    servo_init();             /* Timer1 4-servo scheduler, all -> 90 */
    led_init();               /* onboard LED heartbeat, start ON */
    systick_init();           /* Timer2 1 ms tick */
    joystick_init();          /* ADC + hardware joystick scan (enabled) */
    ir_ctrl_init();           /* NEC IR receiver on PD2 (enabled) + seq engine */
    arm_init();               /* app model reset to 90, MODE_HOLD */
    sei();                    /* enable global interrupts (servo/IR/timer) */

    uart_puts(PSTR("\r\n[meArm] bare-metal AVR ready, servos reset to 90\r\n"));
    uart_puts(PSTR("        type HELP for commands\r\n"));
    arm_status();

    uint32_t last_slow = 0;
    uint32_t last_led  = 0;
    for (;;) {
        /* fast path: never blocks, so IR/serial/sequence events are handled
           with minimal latency */
        cmd_poll();          /* process incoming serial commands */
        ir_ctrl_poll();      /* hardware IR remote -> nudge / seq / stop */
        ir_seq_tick();       /* drive running action set (tick-based) */

        uint32_t now = systick_ms();
        if (now - last_slow >= SLOW_MS) {
            last_slow = now;
            arm_tick();      /* ramp / auto-sweep */
            joystick_scan(); /* hardware joystick -> arm_nudge (if enabled) */
        }
        if (now - last_led >= LED_MS) {
            last_led = now;
            led_toggle();
        }
    }
    return 0;
}
