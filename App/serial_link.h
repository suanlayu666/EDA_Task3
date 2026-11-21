#ifndef SERIAL_LINK_H
#define SERIAL_LINK_H
#include "scope.h"

typedef void (*SerialCommandHandler)(const char *line, uint32_t now);

/* Start continuous receive DMA and service completed bytes outside the interrupts. */
void Serial_Init(void);
void Serial_Poll(uint32_t now, SerialCommandHandler handler);

/* Switch continuous binary output to one or two channels, or turn it off. */
bool Serial_SetStream(unsigned channels);
void Serial_StreamBlock(const SampleBlock *block);
bool Serial_SendText(const char *text);
bool Serial_SendFrame(const ScopeFrame *frame, unsigned channels);
unsigned Serial_StreamChannels(void);
uint32_t Serial_Dropped(void);
uint32_t Serial_RxErrors(void);
#endif
