#ifndef COMMANDS_H
#define COMMANDS_H
#include <stdint.h>

/* Apply one complete command to the same state used by the touch screen. */
void Commands_Execute(const char *line, uint32_t now);
#endif
