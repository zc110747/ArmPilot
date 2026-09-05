#ifndef CORE_IR_CTRL_H
#define CORE_IR_CTRL_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Infrared remote control. Polls the NEC decoder (bsp/ir) and applies the
   same 8-button mapping as the reference sketch:
     左/右   -> base(9)  +2 / -2
     数字2/8 -> left(8)  +2 / -2
     上/下   -> right(7) +2 / -2
     数字4/6 -> grip(6)  +2 / -2                                        */

void ir_ctrl_init(void);
void ir_ctrl_poll(void);          /* call from main loop; applies nudges */
void ir_ctrl_set_enabled(bool on);
bool ir_ctrl_is_enabled(void);

#ifdef __cplusplus
}
#endif

#endif /* CORE_IR_CTRL_H */
