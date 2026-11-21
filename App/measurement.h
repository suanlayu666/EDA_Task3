#ifndef MEASUREMENT_H
#define MEASUREMENT_H
#include <stdbool.h>
#include <stdint.h>

typedef enum { WAVE_DC, WAVE_SINE, WAVE_SQUARE, WAVE_TRIANGLE, WAVE_UNKNOWN } WaveType;

typedef struct {
    float frequency_hz;
    float peak_to_peak_mv;
    float dc_mv;
    float center_mv;
    float first_edge;
    float last_edge;
    unsigned periods;
    WaveType type;
} Measurement;

typedef struct {
    uint32_t sequence;
    uint32_t sample_start;
    Measurement channel[2];
    float phase_deg;
    bool phase_valid;
} MeasurementSet;

typedef uint32_t (*SampleReader)(uint32_t index);
/* Scan real input in small slices so acquisition can keep running. */
void Measurement_Start(SampleReader sample, uint32_t first, unsigned count,
                       unsigned sample_hz, float mv_per_code);
bool Measurement_Poll(MeasurementSet *result);
bool Measurement_Busy(void);
void Measurement_Cancel(void);
const char *Measurement_Name(WaveType type);
#endif
