#ifndef APP_H
#define APP_H

/* Bring up the scope after CubeMX initializes the chip. */
void App_Init(void);
/* Run one cooperative pass of acquisition, serial, analysis and drawing. */
void App_Poll(void);

#endif
