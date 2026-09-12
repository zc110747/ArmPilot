#ifndef BSP_EEPROM_H
#define BSP_EEPROM_H

#include <stdint.h>

/* Minimal AVR EEPROM wrapper (ATmega328P has 1 KB). Used to persist the
   user-learned IR remote codes for the action sets so they survive reset. */

uint32_t ee_get_u32(uint16_t off);   /* read 4 bytes at byte offset */
void     ee_put_u32(uint16_t off, uint32_t v); /* write (wear-levelled update) */

#endif /* BSP_EEPROM_H */
