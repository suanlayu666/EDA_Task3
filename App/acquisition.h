#ifndef ACQUISITION_H
#define ACQUISITION_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    const uint32_t *data;
    uint32_t first;
    uint16_t count;
    bool discontinuity;
} SampleBlock;

void Acquisition_Start(void);
bool Acquisition_Poll(SampleBlock *block);
uint32_t Acquisition_End(void);
uint32_t Acquisition_Count(void);
uint32_t Acquisition_Sample(uint32_t index);
uint32_t Acquisition_Dropped(void);
uint32_t Acquisition_Errors(void);

/* 从一对同步样本中取出指定通道的 12 位值。 */
static inline uint16_t Acquisition_Code(uint32_t sample, unsigned channel)
{
    return (uint16_t)(sample >> (channel * 16U)) & 4095U;
}
#endif
