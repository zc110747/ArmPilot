#include "ir_ctrl.h"

#include <avr/pgmspace.h>
#include "bsp/ir.h"
#include "bsp/uart.h"
#include "bsp/eeprom.h"
#include "core/arm_control.h"
#include "core/ir_seq.h"

static bool g_enabled = true;

/* ---- action-set buttons (1/3/5/7/9) -------------------------------------
   The reference sketch only defined 8 nudge keys (left/right/2/8/up/down/4/6).
   Its remote's number keys (1/3/5/7/9) emit NEC codes that differ per remote,
   so we ship FACTORY DEFAULTS (captured from the user's real remote) AND still
   let the user override them live via `IRLEARN <slot>` (stored in EEPROM).
   On boot: an EEPROM value wins; if EEPROM is erased (0xFFFFFFFF) we fall back
   to the factory default. Slots 1/3/7/9 start a sequence; slot 5 stops it. */
#define IR_EEPROM_BASE 0u          /* 5 * 4 bytes = 20 B of the 1 KB EEPROM */
#define IR_LEARN_NONE  0xFF

static uint32_t g_learned[10];     /* indexed by slot number 1/3/5/7/9; 0 = unset */
static uint8_t  g_learn_arm = IR_LEARN_NONE;

static const uint8_t LEARN_SLOTS[5] PROGMEM = {1, 3, 5, 7, 9};
static const uint8_t SEQ_SLOTS[4]   PROGMEM = {1, 3, 7, 9};

typedef enum { IR_OP_NUDGE = 0 } ir_op_t;

/* 8 original nudge buttons. Codes are copied verbatim from the reference
   sketch's decodedRawData (NEC: address<<24 | ~address<<16 | command<<8 |
   ~command) and verified by tools/test_ir_decode.py. */
static const struct {
    uint32_t code;
    char     name[12];
    uint8_t  op;
    uint8_t  id;    /* nudge target servo (0 if unused) */
    int8_t   d;     /* nudge step (0 if unused)         */
} IR_TAB[8] PROGMEM = {
    {0x00FF08F7u, "left",    IR_OP_NUDGE, 9, +2},
    {0x00FF5AA5u, "right",   IR_OP_NUDGE, 9, -2},
    {0x00FF46B9u, "digit2", IR_OP_NUDGE, 8, +2},
    {0x00FF15EAu, "digit8", IR_OP_NUDGE, 8, -2},
    {0x00FF18E7u, "up",      IR_OP_NUDGE, 7, +2},
    {0x00FF52ADu, "down",    IR_OP_NUDGE, 7, -2},
    {0x00FF44BBu, "digit4", IR_OP_NUDGE, 6, +2},
    {0x00FF43BCu, "digit6", IR_OP_NUDGE, 6, -2},
};

/* Factory defaults for the action-set slots, captured from the user's real
   NEC remote (address 0x00). Index matches LEARN_SLOTS/SEQ_SLOTS order
   {1,3,5,7,9}. Slot 9 is left unbound (0xFFFFFFFF) until its code is known
   or learned via IRLEARN 9. */
static const uint32_t IR_DEFAULTS[5] PROGMEM = {
    0x00FF45BAu,  /* slot 1: 动作集1 (抓取放置) */
    0x00FF47B8u,  /* slot 3: 动作集3 (左右摇摆) */
    0x00FF40BFu,  /* slot 5: 停止            */
    0x00FF07F8u,  /* slot 7: 动作集7 (俯仰)  */
    0x00FF09F6u,  /* slot 9: 动作集9 (开合旋转) */
};

static uint8_t slot_to_idx(uint8_t slot) {
    switch (slot) {
        case 1: return 0;
        case 3: return 1;
        case 5: return 2;
        case 7: return 3;
        case 9: return 4;
    }
    return 0xFF;
}

void ir_ctrl_init(void) {
    ir_init();
    ir_seq_init();
    for (uint8_t i = 0; i < 5; i++) {
        uint8_t  slot = pgm_read_byte(&LEARN_SLOTS[i]);
        uint32_t e    = ee_get_u32(IR_EEPROM_BASE + slot_to_idx(slot) * 4u);
        /* EEPROM erased (0xFFFFFFFF) -> use factory default; otherwise the
           learned (or previously stored) value wins. */
        g_learned[slot] = (e == 0xFFFFFFFFu)
                        ? pgm_read_dword(&IR_DEFAULTS[i])
                        : e;
    }
    g_learn_arm = IR_LEARN_NONE;
    g_enabled   = true;
}

void ir_ctrl_set_enabled(bool on) {
    g_enabled = on;
    if (on) ir_flush();   /* drop any code that arrived while disabled */
}

bool ir_ctrl_is_enabled(void) { return g_enabled; }

/* Bind the next real IR press to an action-set slot (1/3/5/7/9). */
bool ir_ctrl_learn_arm(uint8_t slot) {
    if (slot_to_idx(slot) == 0xFF) return false;
    g_learn_arm = slot;
    return true;
}

/* Erase learned overrides: restore the factory defaults in RAM and mark the
   EEPROM slots erased (0xFFFFFFFF) so a reboot reloads the defaults too. */
void ir_ctrl_clear_learned(void) {
    for (uint8_t i = 0; i < 5; i++) {
        uint8_t  slot = pgm_read_byte(&LEARN_SLOTS[i]);
        g_learned[slot] = pgm_read_dword(&IR_DEFAULTS[i]);
        ee_put_u32(IR_EEPROM_BASE + slot_to_idx(slot) * 4u, 0xFFFFFFFFu);
    }
    g_learn_arm = IR_LEARN_NONE;
}

/* Print the current learned bindings (debug / verification). */
void ir_ctrl_dump_learned(void) {
    uart_puts(PSTR("IR learned bindings:\r\n"));
    for (uint8_t i = 0; i < 5; i++) {
        uint8_t  slot = pgm_read_byte(&LEARN_SLOTS[i]);
        uint32_t c    = g_learned[slot];
        if (c == 0xFFFFFFFFu)
            uart_printf(PSTR("  %u = <unset>\r\n"), (unsigned)slot);
        else
            uart_printf(PSTR("  %u = %08lX\r\n"), (unsigned)slot, c);
    }
}

/* Apply one NEC code: first the 8 hardcoded nudge buttons, then any learned
   action-set slot. Shared by ir_ctrl_poll() (real hardware) and the serial
   `IR <hex>` command, so both paths behave identically. Returns true if the
   code was handled. */
bool ir_ctrl_dispatch(uint32_t code) {
    for (uint8_t i = 0; i < 8; i++) {
        if (pgm_read_dword(&IR_TAB[i].code) != code) continue;

        uint8_t  id = pgm_read_byte(&IR_TAB[i].id);
        int8_t   d  = (int8_t)pgm_read_byte(&IR_TAB[i].d);
        arm_nudge(id, d);
        uart_puts(PSTR("OK IR "));
        uart_puts((PGM_P)&IR_TAB[i].name[0]);
        uart_printf(PSTR(" S%u=%u\r\n"), (unsigned)id,
                    (unsigned)arm_get_angle(id));
        return true;
    }

    /* learned action sets: 1/3/7/9 -> start sequence, 5 -> stop */
    for (uint8_t i = 0; i < 4; i++) {
        uint8_t slot = pgm_read_byte(&SEQ_SLOTS[i]);
        if (g_learned[slot] == code) {
            ir_seq_trigger(slot);
            return true;
        }
    }
    if (g_learned[5] == code) {
        ir_seq_stop();
        return true;
    }
    return false;
}

void ir_ctrl_poll(void) {
    if (!g_enabled) return;
    uint32_t code;
    if (!ir_get_code(&code)) return;

    /* 异步事件：硬件 IR 帧回显。以 "# " 前缀标记，说明它不是命令应答，
       避免上位机命令-应答门控把硬件遥控按键误判为串口指令的应答。 */
    uart_printf(PSTR("# IR RAW=%08lX\r\n"), code);

    if (!ir_ctrl_feed(code))
        uart_puts(PSTR("# IR ? (unbound code)\r\n"));
}

/* Shared by the hardware poll (ir_ctrl_poll) and the serial `IR <hex>` command.
   When a learn slot is armed (IRLEARN) the next code is bound to it (and
   persisted to EEPROM); otherwise the code is dispatched to a known action. */
bool ir_ctrl_feed(uint32_t code) {
    if (g_learn_arm != IR_LEARN_NONE) {
        uint8_t slot = g_learn_arm;
        g_learn_arm = IR_LEARN_NONE;
        g_learned[slot] = code;
        ee_put_u32(IR_EEPROM_BASE + slot_to_idx(slot) * 4u, code);
        uart_printf(PSTR("OK IRLRN slot %u = %08lX (saved)\r\n"),
                    (unsigned)slot, code);
        return true;
    }
    return ir_ctrl_dispatch(code);
}
