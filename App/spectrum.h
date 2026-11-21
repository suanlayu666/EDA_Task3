#ifndef SPECTRUM_H
#define SPECTRUM_H
#include "measurement.h"

enum { SPECTRUM_POINTS = 4096 };

typedef struct {
    float harmonic_mv[5];
    float thd_percent;
    bool valid;
} Spectrum;

void Spectrum_Start(SampleReader sample, uint32_t first, unsigned sample_hz,
                    float mv_per_code, const Measurement *reference);
bool Spectrum_Poll(Spectrum *result);
bool Spectrum_Busy(void);
void Spectrum_Cancel(void);
#endif
