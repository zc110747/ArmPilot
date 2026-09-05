#include "ir_ctrl.h"

#include <avr/pgmspace.h>
#include "bsp/ir.h"
#include "bsp/uart.h"
#include "core/arm_control.h"

static bool g_enabled = true;

/* {NEC 32-bit code, button name, target servo id, step} -- from the
   reference sketch's loop(): 8 buttons, each nudges its servo by +/-2 deg.
   Stored in flash (PROGMEM). The button name is an inline char[] in the
   flash struct, so uart_puts() can emit it directly without copying to RAM. */
static const struct {
    uint32_t code;
    char     name[12];
    uint8_t  id;
    int8_t   d;
} IR_TAB[8] PROGMEM = {
    {0xF708FF00u, "左",   9, +2}, {0xA55AFF00u, "右",   9, -2},
    {0xB946FF00u, "数字2", 8, +2}, {0xEA15FF00u, "数字8", 8, -2},
    {0xE718FF00u, "上",   7, +2}, {0xAD52FF00u, "下",   7, -2},
    {0xBB44FF00u, "数字4", 6, +2}, {0xBC43FF00u, "数字6", 6, -2},
};

void ir_ctrl_init(void) {
    ir_init();
    g_enabled = true;
}

void ir_ctrl_set_enabled(bool on) { g_enabled = on; }
bool ir_ctrl_is_enabled(void)     { return g_enabled; }

void ir_ctrl_poll(void) {
    if (!g_enabled) return;
    uint32_t code;
    if (!ir_get_code(&code)) return;

    for (uint8_t i = 0; i < 8; i++) {
        if (pgm_read_dword(&IR_TAB[i].code) == code) {
            uint8_t  id = pgm_read_byte(&IR_TAB[i].id);
            int8_t   d  = (int8_t)pgm_read_byte(&IR_TAB[i].d);
            arm_nudge(id, d);
            uart_puts(PSTR("IR "));
            uart_puts((PGM_P)&IR_TAB[i].name[0]);
            uart_printf(PSTR(" S%u=%u\r\n"), (unsigned)id,
                        (unsigned)arm_get_angle(id));
            return;
        }
    }
    /* unknown NEC code: ignore (original only acted on the 8 known codes) */
}
