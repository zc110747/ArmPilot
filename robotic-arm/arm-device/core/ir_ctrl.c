#include "ir_ctrl.h"

#include <avr/pgmspace.h>
#include "bsp/ir.h"
#include "bsp/uart.h"
#include "core/arm_control.h"
#include "core/ir_seq.h"

static bool g_enabled = true;

typedef enum { IR_OP_NUDGE = 0, IR_OP_SEQ = 1, IR_OP_STOP = 2 } ir_op_t;

/* NEC 32-bit codes (address<<24 | ~address<<16 | command<<8 | ~command).
   The 8 original buttons (from the reference sketch) nudge a servo by +/-2.
   Buttons 1/3/7/9 start an action set (ir_seq_trigger); button 5 stops the
   loop (req 3). New codes use distinct addresses (0xC1/0xC3/0xC5/0xC7/0xC9)
   so they never collide with the original set. All in flash (PROGMEM); the
   button name is an inline char[] so uart_puts() can emit it from flash. */
static const struct {
    uint32_t code;
    char     name[12];
    uint8_t  op;
    uint8_t  id;    /* nudge target servo (0 if unused) */
    int8_t   d;     /* nudge step (0 if unused)         */
    uint8_t  seq;   /* sequence button 1/3/7/9 (0 if unused) */
} IR_TAB[13] PROGMEM = {
    /* ---- 8 original nudge buttons ---- */
    {0xF708FF00u, "左",    IR_OP_NUDGE, 9, +2, 0},
    {0xA55AFF00u, "右",    IR_OP_NUDGE, 9, -2, 0},
    {0xB946FF00u, "数字2", IR_OP_NUDGE, 8, +2, 0},
    {0xEA15FF00u, "数字8", IR_OP_NUDGE, 8, -2, 0},
    {0xE718FF00u, "上",    IR_OP_NUDGE, 7, +2, 0},
    {0xAD52FF00u, "下",    IR_OP_NUDGE, 7, -2, 0},
    {0xBB44FF00u, "数字4", IR_OP_NUDGE, 6, +2, 0},
    {0xBC43FF00u, "数字6", IR_OP_NUDGE, 6, -2, 0},
    /* ---- new action-set buttons (1/3/7/9) ---- */
    {0xC13E01FEu, "1", IR_OP_SEQ, 0, 0, 1},
    {0xC33C03FCu, "3", IR_OP_SEQ, 0, 0, 3},
    {0xC73807F8u, "7", IR_OP_SEQ, 0, 0, 7},
    {0xC93609F6u, "9", IR_OP_SEQ, 0, 0, 9},
    /* ---- stop button (5) ---- */
    {0xC53A05FAu, "5", IR_OP_STOP, 0, 0, 0},
};

void ir_ctrl_init(void) {
    ir_init();
    ir_seq_init();
    g_enabled = true;
}

void ir_ctrl_set_enabled(bool on) { g_enabled = on; }
bool ir_ctrl_is_enabled(void)     { return g_enabled; }

/* Look up a decoded NEC code and act on it. Shared by ir_ctrl_poll() (real
   hardware) and the serial `IR <hex>` command (cmd.c), so both paths behave
   identically. Returns true if the code was handled. */
bool ir_ctrl_dispatch(uint32_t code) {
    for (uint8_t i = 0; i < 13; i++) {
        if (pgm_read_dword(&IR_TAB[i].code) != code) continue;

        uint8_t op = pgm_read_byte(&IR_TAB[i].op);
        if (op == IR_OP_NUDGE) {
            uint8_t  id = pgm_read_byte(&IR_TAB[i].id);
            int8_t   d  = (int8_t)pgm_read_byte(&IR_TAB[i].d);
            arm_nudge(id, d);
            uart_puts(PSTR("OK IR "));
            uart_puts((PGM_P)&IR_TAB[i].name[0]);
            uart_printf(PSTR(" S%u=%u\r\n"), (unsigned)id,
                        (unsigned)arm_get_angle(id));
        } else if (op == IR_OP_SEQ) {
            ir_seq_trigger(pgm_read_byte(&IR_TAB[i].seq));
        } else { /* IR_OP_STOP */
            ir_seq_stop();
        }
        return true;
    }
    return false;
}

void ir_ctrl_poll(void) {
    if (!g_enabled) return;
    uint32_t code;
    if (ir_get_code(&code))
        ir_ctrl_dispatch(code);   /* unknown codes are silently ignored */
}
