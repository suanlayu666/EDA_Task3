#ifndef SCOPE_H
#define SCOPE_H
#include "acquisition.h"
#include "measurement.h"
#include "spectrum.h"
#include "scope_config.h"

typedef enum { SCOPE_AUTO, SCOPE_NORMAL, SCOPE_SINGLE } TriggerMode;
typedef enum { SCOPE_RISING, SCOPE_FALLING } TriggerEdge;
typedef enum { SCOPE_RUNNING, SCOPE_ARMED, SCOPE_FROZEN } RunState;

typedef struct {
    unsigned time_us_div;
    unsigned vertical_mv_div;
    unsigned trigger_mv;
    unsigned reference_mv;
    TriggerMode mode;
    TriggerEdge edge;
    RunState state;
    bool dual;
    bool auto_range;
} ScopeSettings;

typedef struct {
    uint32_t sequence;
    uint32_t captured_at_ms;
    unsigned count;
    unsigned reference_mv;
    unsigned time_us_div;
    float start_fraction;
    bool triggered;
    MeasurementSet measurement;
    Spectrum spectrum;
    uint8_t samples[SCOPE_FRAME_CAPACITY][3];
} ScopeFrame;

/* 从一对压缩存储的采样中无损取出指定通道的 12 位值。 */
static inline uint16_t Scope_FrameCode(const ScopeFrame *frame, unsigned index, unsigned channel)
{
    const uint8_t *sample = frame->samples[index];
    if (channel == 0) return sample[0] | ((sample[1] & 15U) << 8U);
    return (sample[1] >> 4U) | (sample[2] << 4U);
}

/* 在非空捕获帧中插值，末尾直接取最后一个样点。 */
static inline float Scope_Interpolate(const ScopeFrame *frame, float index, unsigned channel)
{
    unsigned i = (unsigned)index;
    if (i >= frame->count - 1U) return Scope_FrameCode(frame, frame->count - 1U, channel);
    float a = Scope_FrameCode(frame, i, channel);
    float b = Scope_FrameCode(frame, i + 1U, channel);
    return a + (index - i) * (b - a);
}

void Scope_Init(void);
const ScopeSettings *Scope_Settings(void);
const ScopeFrame *Scope_Frame(void);
const char *Scope_ModeName(void);
const char *Scope_StateName(void);
bool Scope_SetReference(unsigned mv);
bool Scope_SetTime(unsigned us_div);
bool Scope_SetVertical(unsigned mv_div);
bool Scope_SetTrigger(unsigned mv);
void Scope_SetMode(TriggerMode mode, uint32_t now);
void Scope_SetEdge(TriggerEdge edge);
void Scope_SetDual(bool dual);
void Scope_SetAutoRange(bool auto_range);
void Scope_Run(uint32_t now);
void Scope_Stop(void);
void Scope_CycleTime(void);
void Scope_CycleVertical(void);
void Scope_Consume(const SampleBlock *block);
void Scope_Poll(uint32_t now);
#endif
