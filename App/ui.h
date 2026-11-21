#ifndef SCOPE_UI_H
#define SCOPE_UI_H
#include <stdbool.h>
#include <stdint.h>

/* 初始化 LCD 和触摸，必要时开始四点校准。 */
void Ui_Init(void);
void Ui_Poll(uint32_t now);
void Ui_SetSpectrum(bool visible);
void Ui_RequestCalibration(void);
unsigned Ui_Fps(void);
unsigned Ui_LcdId(void);
bool Ui_ScanSynchronized(void);
uint32_t Ui_ScanMisses(void);
bool Ui_TouchReady(void);
bool Ui_IsSpectrum(void);
#endif
