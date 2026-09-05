#ifndef CORE_CMD_H
#define CORE_CMD_H

#include <stdbool.h>

/* Poll the UART ring for a complete line and process one command if present.
   Must be called frequently from the main loop. */
void cmd_poll(void);

#endif /* CORE_CMD_H */
