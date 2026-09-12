#ifndef CORE_IR_CTRL_H
#define CORE_IR_CTRL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Infrared remote control. Decodes NEC frames (bsp/ir) and applies them:
     - 8 original buttons (左/右/数字2/8/上/下/数字4/6) nudge a servo +/-2 deg
       (same mapping as the reference sketch)
     - buttons 1/3/7/9 start an action set (core/ir_seq)
     - button 5 stops the running action set
   ir_ctrl_dispatch() is the single entry point, used by both the hardware
   poll (ir_ctrl_poll) and the serial `IR <hex>` command. */

void ir_ctrl_init(void);
void ir_ctrl_poll(void);          /* call from main loop; applies nudges/seqs */
void ir_ctrl_set_enabled(bool on);
bool ir_ctrl_is_enabled(void);

/* Action-set buttons (1/3/5/7/9) are learned from the real remote at runtime
   because their NEC codes are unknown at build time. */
bool ir_ctrl_learn_arm(uint8_t slot);   /* arm: next IR press binds to slot */
void ir_ctrl_clear_learned(void);       /* erase all learned bindings       */
void ir_ctrl_dump_learned(void);        /* print current bindings           */

/* Decode + apply one NEC code. Returns true if it matched a known button. */
bool ir_ctrl_dispatch(uint32_t code);

/* Common entry for one received/synthetic NEC code: if a learn slot is armed
   (IRLEARN) it binds this code, otherwise it dispatches. Shared by the
   hardware poll (ir_ctrl_poll) and the serial `IR <hex>` command so both
   paths support learning identically. */
bool ir_ctrl_feed(uint32_t code);

#ifdef __cplusplus
}
#endif

#endif /* CORE_IR_CTRL_H */
