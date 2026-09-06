#include "eeprom.h"

#include <avr/eeprom.h>
#include <stdint.h>

uint32_t ee_get_u32(uint16_t off) {
    return eeprom_read_dword((const uint32_t *)(uintptr_t)off);
}

void ee_put_u32(uint16_t off, uint32_t v) {
    eeprom_update_dword((uint32_t *)(uintptr_t)off, v);
}
