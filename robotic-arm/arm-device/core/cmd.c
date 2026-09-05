#include "cmd.h"
#include "arm_control.h"
#include "core/joystick.h"
#include "core/ir_ctrl.h"
#include "core/ir_seq.h"
#include "bsp/uart.h"
#include "bsp/servo.h"
#include "bsp/adc.h"

#include <avr/pgmspace.h>
#include <string.h>
#include <ctype.h>

/* ---- serial protocol -----------------------------------------------------
   SET  <id> <angle>            single motor to angle (MODE_HOLD)
   SET  <id> <angle> [id ang]... up to 3 motors in one command (combined)
   S<id>=<angle>               shorthand single-set, e.g. S9=90
   STOP <id>                   freeze auto-sweep, hold current angle
   AUTO <id>                   joystick-like self-sweep between limits
   JOY  <raw9> <raw8> <raw6> <raw7>   full joystick frame (4 analog raw 0..1023)
   JOY  <id> <raw>             single-axis joystick nudge (id 6/7/8/9)
   IR   <hexcode>              8-button IR remote (NEC 32-bit code)
   RESET                       all servos -> 90
   STATUS | ?                  print S6..S9 angles + modes
   HELP                        print this help
   Rules: SET may address at most 3 motors. left(8) and right(7) MAY be
   combined (user: 左右舵允许同时工作). JOY/IR exist both as serial commands
   (manual test) AND as real hardware: joystick_scan() reads ADC A0..A3,
   ir_ctrl_poll() decodes NEC on PD2 -- both mirror the original Arduino
   handleJoystickControl()/IR recv logic (thresholds & button map identical).
   IR buttons 1/3/7/9 start ~15 s action sets (5 s mid-pause, loop); button 5
   or any JOY command stops the loop; another 1/3/7/9 switches the set. The
   SEQ command drives the same sets over serial for testing.
   JOYHW/IRHW toggle the hardware paths; ADC prints raw axis values.
   All literals are PSTR()'d so avr-gcc keeps them in flash, not RAM.        */

#define LINE_SZ 64
#define MAX_PAIRS 3

static char line[LINE_SZ];
static uint8_t line_len = 0;

/* tiny atoi for unsigned */
static uint8_t parse_u8(const char *s, bool *ok) {
    *ok = false;
    if (!s || !*s) return 0;
    int v = 0;
    for (const char *p = s; *p; p++) {
        if (!isdigit((unsigned char)*p)) return 0;
        v = v * 10 + (*p - '0');
        if (v > 999) return 0;
    }
    *ok = true;
    return (uint8_t)v;
}

/* signed integer parser (for joystick raw 0..1023) */
static int parse_int(const char *s, bool *ok) {
    *ok = false;
    if (!s || !*s) return 0;
    const char *p = s;
    int sign = 1;
    if (*p == '-') { sign = -1; p++; }
    else if (*p == '+') { p++; }
    int v = 0, started = 0;
    for (; *p; p++) {
        if (!isdigit((unsigned char)*p)) return 0;
        v = v * 10 + (*p - '0');
        started = 1;
        if (v > 1023) return 0; /* 10-bit ADC range (joystick raw max) */
    }
    if (!started) return 0;
    *ok = true;
    return sign * v;
}

/* parse up to 8 hex digits (optional 0x prefix) into a 32-bit value */
static bool parse_hex32(const char *s, uint32_t *out) {
    if (!s || !*s) return false;
    if (s[0] == '0' && (s[1] == 'x' || s[1] == 'X')) s += 2;
    uint32_t v = 0;
    uint8_t n = 0;
    for (const char *p = s; *p; p++) {
        char c = *p;
        uint8_t d;
        if      (c >= '0' && c <= '9') d = (uint8_t)(c - '0');
        else if (c >= 'a' && c <= 'f') d = (uint8_t)(c - 'a' + 10);
        else if (c >= 'A' && c <= 'F') d = (uint8_t)(c - 'A' + 10);
        else return false;
        v = (v << 4) | d;
        if (++n > 8) return false; /* too long */
    }
    if (n == 0) return false;
    *out = v;
    return true;
}

/* Per-axis sense lives in core/joystick.c (joystick_delta), shared by the
   hardware joystick scan and the JOY serial command so they stay in sync
   with the original handleJoystickControl() mapping. */

/* split a string into at most n tokens (space/comma separated) */
static uint8_t tokenize(char *src, char *tok[], uint8_t maxn) {
    uint8_t n = 0;
    char *p = src;
    while (*p && n < maxn) {
        while (*p == ' ' || *p == ',' || *p == '\t') *p++ = '\0';
        if (!*p) break;
        tok[n++] = p;
        while (*p && *p != ' ' && *p != ',' && *p != '\t') p++;
    }
    return n;
}

static void send_help(void) {
    uart_puts(PSTR("meArm commands:\r\n"));
    uart_puts(PSTR("  SET <id> <angle>           move one servo (id 6/7/8/9)\r\n"));
    uart_puts(PSTR("  SET <id> <ang> [id ang]..  combined, <=3 motors (7&8 allowed)\r\n"));
    uart_puts(PSTR("  S<id>=<angle>             shorthand, e.g. S9=90\r\n"));
    uart_puts(PSTR("  STOP <id>                 freeze auto-sweep, hold angle\r\n"));
    uart_puts(PSTR("  AUTO <id>                 self-sweep between limits\r\n"));
    uart_puts(PSTR("  JOY <r9> <r8> <r6> <r7>    joystick frame (raw 0..1023, +/-1 each)\r\n"));
    uart_puts(PSTR("  JOY <id> <raw>             single-axis joystick nudge\r\n"));
    uart_puts(PSTR("  IR <hex>                   8-btn remote (F708FF00..) + seq 1/3/7/9 + stop 5\r\n"));
    uart_puts(PSTR("  SEQ 1|3|7|9                run action set (switch if another runs)\r\n"));
    uart_puts(PSTR("  SEQ STOP                   stop running action set (same as IR 5)\r\n"));
    uart_puts(PSTR("  SEQ ?                      report running set / idle\r\n"));
    uart_puts(PSTR("  JOYHW ON|OFF               hardware joystick scan enable\r\n"));
    uart_puts(PSTR("  IRHW ON|OFF                hardware IR receiver enable\r\n"));
    uart_puts(PSTR("  ADC                        print A0..A3 raw values (debug)\r\n"));
    uart_puts(PSTR("  RESET                      all -> 90 deg\r\n"));
    uart_puts(PSTR("  STATUS | ?                report angles + modes\r\n"));
    uart_puts(PSTR("  HELP                       this text\r\n"));
}

/* apply a SET with k (id,angle) pairs; enforces <=3 motors per command.
   NOTE: left(8) and right(7) MAY be combined now (user: 左右舵允许同时工作). */
static void do_set(uint8_t ids[], uint8_t angles[], uint8_t k) {
    if (k == 0) { uart_puts(PSTR("ERR SYNTAX\r\n")); return; }
    if (k > MAX_PAIRS) { uart_puts(PSTR("ERR TOO_MANY (max 3)\r\n")); return; }

    for (uint8_t i = 0; i < k; i++) {
        if (!arm_id_valid(ids[i])) { uart_printf(PSTR("ERR BAD_ID S%u\r\n"), ids[i]); return; }
        angles[i] = arm_set_angle(ids[i], angles[i]); /* report applied (clamped) */
    }

    uart_puts(PSTR("OK SET"));
    for (uint8_t i = 0; i < k; i++)
        uart_printf(PSTR(" S%u=%u"), ids[i], angles[i]);
    uart_puts(PSTR("\r\n"));
}

static void process_line(char *buf) {
    char *tok[16];
    uint8_t n = tokenize(buf, tok, 16);
    if (n == 0) return;

    /* verb upper (in place) */
    for (char *c = tok[0]; *c; c++) *c = (char)toupper((unsigned char)*c);
    char *verb = tok[0];

    if (strcmp(verb, "HELP") == 0) { send_help(); return; }
    if (strcmp(verb, "?") == 0)    { arm_status(); return; }
    if (strcmp(verb, "STATUS") == 0) { arm_status(); return; }
    if (strcmp(verb, "RESET") == 0)  { arm_reset(); uart_puts(PSTR("OK RESET -> 90\r\n")); arm_status(); return; }

    if (strcmp(verb, "STOP") == 0) {
        if (n < 2) { uart_puts(PSTR("ERR SYNTAX\r\n")); return; }
        bool ok; uint8_t id = parse_u8(tok[1], &ok);
        if (!ok || !arm_id_valid(id)) { uart_printf(PSTR("ERR BAD_ID S%s\r\n"), tok[1]); return; }
        arm_stop(id);
        uart_printf(PSTR("OK STOP S%u (hold %u)\r\n"), id, arm_get_angle(id));
        return;
    }
    if (strcmp(verb, "AUTO") == 0) {
        if (n < 2) { uart_puts(PSTR("ERR SYNTAX\r\n")); return; }
        bool ok; uint8_t id = parse_u8(tok[1], &ok);
        if (!ok || !arm_id_valid(id)) { uart_printf(PSTR("ERR BAD_ID S%s\r\n"), tok[1]); return; }
        if (!arm_auto(id)) { uart_printf(PSTR("ERR AUTO S%u\r\n"), id); return; }
        uart_printf(PSTR("OK AUTO S%u\r\n"), id);
        return;
    }

    /* ---- JOY: emulate original Arduino joystick (handleJoystickControl) ---- */
    if (strcmp(verb, "JOY") == 0) {
        if (n == 3) {
            /* single axis: JOY <id> <raw> */
            bool ok1, ok2;
            uint8_t id = parse_u8(tok[1], &ok1);
            int raw = parse_int(tok[2], &ok2);
            if (!ok1 || !arm_id_valid(id)) { uart_printf(PSTR("ERR BAD_ID S%s\r\n"), tok[1]); return; }
            if (!ok2) { uart_printf(PSTR("ERR RAW %s\r\n"), tok[2]); return; }
            int8_t d = joystick_delta(id, raw);
            ir_seq_stop();   /* a joystick command ends the auto-loop (req 3) */
            arm_nudge(id, d);
            uart_printf(PSTR("OK JOY S%u=%u\r\n"), id, arm_get_angle(id));
            return;
        }
        if (n == 5) {
            /* full frame: JOY <raw9> <raw8> <raw6> <raw7> (A0..A3 on the Uno) */
            static const uint8_t ids[4] PROGMEM = {9, 8, 6, 7};
            int raw[4];
            for (uint8_t i = 0; i < 4; i++) {
                bool ok;
                raw[i] = parse_int(tok[1 + i], &ok);
                if (!ok) { uart_printf(PSTR("ERR RAW %s\r\n"), tok[1 + i]); return; }
            }
            for (uint8_t i = 0; i < 4; i++) {
                uint8_t id = pgm_read_byte(&ids[i]);
                int8_t d = joystick_delta(id, raw[i]);
                if (d) arm_nudge(id, d);
            }
            ir_seq_stop();   /* a joystick command ends the auto-loop (req 3) */
            uart_printf(PSTR("OK JOY S6=%u S7=%u S8=%u S9=%u\r\n"),
                arm_get_angle(6), arm_get_angle(7),
                arm_get_angle(8), arm_get_angle(9));
            return;
        }
        uart_puts(PSTR("ERR SYNTAX\r\n"));
        return;
    }

    /* ---- IR: emulate original Arduino IR remote (8 buttons + seq 1/3/7/9 + stop 5)
       Routed through ir_ctrl_dispatch() so the serial command behaves exactly
       like a real NEC frame from the hardware receiver. ----------------------- */
    if (strcmp(verb, "IR") == 0) {
        if (n < 2) { uart_puts(PSTR("ERR SYNTAX\r\n")); return; }
        uint32_t code;
        if (!parse_hex32(tok[1], &code)) { uart_printf(PSTR("ERR IR HEX %s\r\n"), tok[1]); return; }
        if (!ir_ctrl_dispatch(code))
            uart_printf(PSTR("ERR IR UNKNOWN %08lX\r\n"), code);
        return;
    }

    /* ---- SEQ: run / stop the IR action sets without a remote (testing) -----
       SEQ 1|3|7|9  start that button's sequence (switch task if another runs)
       SEQ STOP      end the running loop (same as IR button 5)
       SEQ ?         report current running set                              */
    if (strcmp(verb, "SEQ") == 0) {
        if (n < 2) { uart_puts(PSTR("ERR SYNTAX\r\n")); return; }
        if (strcasecmp(tok[1], "STOP") == 0) {
            ir_seq_stop();
            return;
        }
        if (strcasecmp(tok[1], "?") == 0) {
            if (ir_seq_is_running())
                uart_printf(PSTR("SEQ running %u\r\n"), (unsigned)ir_seq_which());
            else
                uart_puts(PSTR("SEQ idle\r\n"));
            return;
        }
        bool ok; uint8_t which = parse_u8(tok[1], &ok);
        if (!ok || (which != 1 && which != 3 && which != 7 && which != 9)) {
            uart_puts(PSTR("ERR SEQ (1|3|7|9|STOP|?)\r\n"));
            return;
        }
        ir_seq_trigger(which);
        return;
    }

    /* ---- JOYHW / IRHW: enable or disable the hardware control paths ---- */
    if (strcmp(verb, "JOYHW") == 0 || strcmp(verb, "IRHW") == 0) {
        bool is_joy = (verb[0] == 'J');
        if (n < 2) { uart_puts(PSTR("ERR SYNTAX\r\n")); return; }
        bool on;
        if      (strcasecmp(tok[1], "ON")  == 0) on = true;
        else if (strcasecmp(tok[1], "OFF") == 0) on = false;
        else { uart_puts(PSTR("ERR ARG (ON/OFF)\r\n")); return; }
        if (is_joy) joystick_set_enabled(on); else ir_ctrl_set_enabled(on);
        uart_printf(PSTR("OK %s %s\r\n"), verb, on ? "ON" : "OFF");
        return;
    }

    /* ---- ADC: print the 4 joystick axis raw values (debug / wiring) ---- */
    if (strcmp(verb, "ADC") == 0) {
        uart_printf(PSTR("ADC A0=%u A1=%u A2=%u A3=%u\r\n"),
            adc_read(0), adc_read(1), adc_read(2), adc_read(3));
        return;
    }

    if (strcmp(verb, "SET") == 0) {
        if (n < 3 || (n % 2) != 1) { uart_puts(PSTR("ERR SYNTAX\r\n")); return; }
        uint8_t total = (uint8_t)((n - 1) / 2); /* total id/angle pairs */
        if (total > MAX_PAIRS) { uart_puts(PSTR("ERR TOO_MANY (max 3)\r\n")); return; }
        uint8_t ids[MAX_PAIRS], angles[MAX_PAIRS];
        uint8_t k = 0;
        for (uint8_t i = 1; i + 1 < n && k < total; i += 2) {
            bool ok1, ok2;
            uint8_t id = parse_u8(tok[i], &ok1);
            uint8_t ang = parse_u8(tok[i + 1], &ok2);
            if (!ok1 || !arm_id_valid(id)) { uart_printf(PSTR("ERR BAD_ID S%s\r\n"), tok[i]); return; }
            if (!ok2) { uart_printf(PSTR("ERR ANGLE %s\r\n"), tok[i + 1]); return; }
            ids[k] = id; angles[k] = ang; k++;
        }
        do_set(ids, angles, k);
        return;
    }

    /* shorthand S<id>=<angle> */
    if (verb[0] == 'S' && verb[1] >= '6' && verb[1] <= '9' && verb[2] == '=') {
        uint8_t id = (uint8_t)(verb[1] - '0');
        bool ok; uint8_t ang = parse_u8(verb + 3, &ok);
        if (!ok) { uart_printf(PSTR("ERR ANGLE %s\r\n"), verb + 3); return; }
        uint8_t ids[1] = {id}, angles[1] = {ang};
        do_set(ids, angles, 1);
        return;
    }

    uart_printf(PSTR("ERR UNKNOWN %s\r\n"), verb);
}

void cmd_poll(void) {
    int c;
    while ((c = uart_getc_nowait()) >= 0) {
        if (c == '\r' || c == '\n') {
            if (line_len > 0) {
                line[line_len] = '\0';
                process_line(line);
                line_len = 0;
            }
        } else if (line_len < LINE_SZ - 1) {
            line[line_len++] = (char)c;
        }
        /* ignore overflow beyond buffer */
    }
}
