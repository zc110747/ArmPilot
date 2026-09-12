#ifndef CORE_IR_SEQ_H
#define CORE_IR_SEQ_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Infrared "action set" engine (req: buttons 1/3/7/9 each run a ~15 s motion
   sequence with a 5 s pause in the middle, then loop).

     - ir_seq_trigger(1|3|7|9) : start that button's sequence from its first
                                 keyframe (restarts if already running -> switch
                                 task, req 4).
     - ir_seq_stop()            : end the loop (IR button 5, or a joystick
                                 command, req 3).
     - ir_seq_tick()            : call every main-loop iteration. Tick-driven
                                 (no delay); advances keyframes only after the
                                 move has settled AND the keyframe's hold gap
                                 elapsed (req 2), detects stop/switch events.
     - ir_seq_is_running()      : for STATUS reporting.

   Sequences live in flash (PROGMEM); only the small run-state struct is in RAM. */

void ir_seq_init(void);
void ir_seq_trigger(uint8_t which);   /* which in {1,3,7,9} */
void ir_seq_stop(void);
void ir_seq_tick(void);
bool ir_seq_is_running(void);
uint8_t ir_seq_which(void);           /* current button (1/3/7/9) or 0 if idle */

#ifdef __cplusplus
}
#endif

#endif /* CORE_IR_SEQ_H */
