#include "acquisition.h"
#include "measurement.h"
#include "scope.h"
#include "spectrum.h"
#include "stream_encode.h"
#include <assert.h>
#include <math.h>
#include <stdio.h>

/* Mirror the real circular buffer and deliver the same block shape as ADC DMA. */
static uint32_t history[SCOPE_HISTORY_SIZE], block[SCOPE_DMA_HALF], end, available;
static const double tau = 6.283185307179586;

uint32_t Acquisition_Sample(uint32_t index) { return history[index & (SCOPE_HISTORY_SIZE - 1U)]; }
uint32_t Acquisition_End(void) { return end; }
uint32_t Acquisition_Count(void) { return available; }

static double input_signal(double cycles, WaveType type)
{
    double angle = tau * cycles;
    if (type == WAVE_SQUARE) return sin(angle) >= 0 ? 1.0 : -1.0;
    if (type == WAVE_TRIANGLE) return asin(sin(angle)) * 4.0 / tau;
    return sin(angle);
}

static uint32_t sample(double a, double b)
{
    return (uint32_t)lround(a * 4095.0 / 3.3) | ((uint32_t)lround(b * 4095.0 / 3.3) << 16U);
}

static MeasurementSet measure(void)
{
    MeasurementSet result;
    Measurement_Start(Acquisition_Sample, 0, SCOPE_MEASURE_SAMPLES, SCOPE_SAMPLE_HZ, 3300.0f / 4095);
    while (!Measurement_Poll(&result)) {}
    return result;
}

/* Measure both amplitude extremes and all nine required identification cases. */
static void verify_measurement(void)
{
    for (unsigned hz = 1000; hz <= 5000; hz += 1000) {
        for (unsigned shape = WAVE_SINE; shape <= WAVE_TRIANGLE; ++shape) {
            if (shape != WAVE_SINE && hz != 1000 && hz != 2000 && hz != 5000) continue;
            for (unsigned range = 0; range < 2; ++range) {
                double vpp = range ? 2.0 : 0.2, dc = range ? 2.0 : 1.0;
                for (unsigned i = 0; i < SCOPE_MEASURE_SAMPLES; ++i) {
                    double t = (double)i * hz / SCOPE_SAMPLE_HZ + 0.061;
                    history[i] = sample(dc + vpp * 0.5 * input_signal(t, shape),
                                        1.5 + 0.5 * input_signal(t + 0.25, shape));
                }
                MeasurementSet m = measure();
                assert(fabs(m.channel[0].frequency_hz - hz) <= 0.03 * hz);
                assert(fabs(m.channel[0].peak_to_peak_mv / 1000.0 - vpp) <= 0.05 * vpp);
                assert(fabs(m.channel[0].dc_mv / 1000.0 - dc) <= 0.05 * dc);
                assert(m.channel[0].type == shape && m.channel[1].type == shape);
                assert(m.phase_valid && fabs(m.phase_deg - 90) <= 5);
            }
        }
    }
    puts("DSP PASS: frequency, Vpp, DC, nine wave classifications, simultaneous phase");
}

/* Check signed phase handling on both sides of the 0-degree boundary. */
static void verify_phase_angles(void)
{
    const unsigned angles[] = {1, 37, 90, 180, 271, 359};
    for (unsigned hz = 1000; hz <= 5000; hz += 1000) {
        for (unsigned j = 0; j < sizeof angles / sizeof angles[0]; ++j) {
            for (unsigned i = 0; i < SCOPE_MEASURE_SAMPLES; ++i) {
                double phase = tau * (0.031 + (double)i * hz / SCOPE_SAMPLE_HZ);
                history[i] = sample(1.0 + 0.10 * sin(phase),
                                    2.0 + 0.70 * sin(phase + tau * angles[j] / 360.0));
            }
            MeasurementSet m = measure();
            assert(m.phase_valid && m.phase_deg >= 0 && m.phase_deg < 360);
            assert(fabs(remainder(m.phase_deg - angles[j], 360.0)) <= 5);
        }
    }
    puts("DSP PASS: both sides of the 0-degree phase boundary at all five frequencies");
}

/* Compare FFT THD with known harmonic amplitudes at all five test frequencies. */
static void verify_fft(void)
{
    for (unsigned hz = 1000; hz <= 5000; hz += 1000) {
        for (unsigned i = 0; i < SCOPE_MEASURE_SAMPLES; ++i) {
            double a = tau * i * hz / SCOPE_SAMPLE_HZ;
            double v = 1.5 + 0.5 * sin(a) + 0.025 * sin(2 * a + .7) +
                       0.03 * sin(3 * a) + 0.01 * sin(4 * a) + 0.02 * sin(5 * a);
            history[i] = sample(v, v);
        }
        Measurement m = {.frequency_hz = (float)hz, .dc_mv = 1500};
        Spectrum_Start(Acquisition_Sample, 0, SCOPE_SAMPLE_HZ, 3300.0f / 4095, &m);
        Spectrum s;
        while (!Spectrum_Poll(&s)) {}
        double expected = 100.0 * sqrt(.025*.025 + .03*.03 + .01*.01 + .02*.02) / .5;
        assert(s.valid && fabs(s.thd_percent - expected) <= .05 * expected);
        const float mv[] = {500, 25, 30, 10, 20};
        for (unsigned i = 0; i < 5; ++i) assert(fabs(s.harmonic_mv[i] - mv[i]) <= .05 * mv[i]);
    }
    puts("DSP PASS: FFT 1-5 harmonics and THD at 1-5 kHz");
}

static uint32_t now_ms;

/* Step the firmware capture and analysis with the normal five ms DMA cadence. */
static void run_scope(unsigned ms, unsigned hz, double vpp)
{
    uint32_t deadline = now_ms + ms;
    while (now_ms < deadline) {
        for (unsigned i = 0; i < SCOPE_DMA_HALF; ++i) {
            double t = (double)(end + i) * hz / SCOPE_SAMPLE_HZ;
            history[(end + i) & (SCOPE_HISTORY_SIZE - 1U)] = block[i] =
                sample(1.65 + 0.5 * vpp * sin(tau * t), 1.65 + 0.5 * vpp * cos(tau * t));
        }
        SampleBlock samples = {block, end, SCOPE_DMA_HALF, false};
        end += SCOPE_DMA_HALF;
        if (available < SCOPE_HISTORY_SIZE) available += SCOPE_DMA_HALF;
        Scope_Consume(&samples);
        for (unsigned tick = 0; tick < 110; ++tick) Scope_Poll(now_ms);
        now_ms += 5;
    }
}

static void verify_scope(void)
{
    Scope_Init();
    Scope_SetTrigger(1650);
    Scope_SetTime(2000);
    run_scope(200, 100, 1);
    const ScopeFrame *frame = Scope_Frame();
    assert(frame->triggered && frame->count == 4002 && frame->sequence);
    unsigned trigger = (frame->count - 2) / 4U;
    unsigned a = Scope_FrameCode(frame, trigger, 0);
    unsigned b = Scope_FrameCode(frame, trigger + 1, 0);
    assert(a <= 2048 && b >= 2047);
    Scope_SetMode(SCOPE_NORMAL, now_ms);
    Scope_SetTrigger(50);
    unsigned seq = frame->sequence;
    run_scope(300, 100, 1);
    assert(frame->sequence == seq);
    Scope_SetMode(SCOPE_SINGLE, now_ms);
    Scope_SetTrigger(1650);
    run_scope(300, 3000, 1);
    assert(frame->sequence == seq + 1 && Scope_Settings()->state == SCOPE_FROZEN);
    assert(fabs(frame->measurement.channel[0].frequency_hz - 3000) < 90);
    assert(frame->spectrum.valid);
    run_scope(200, 1000, 1);
    assert(frame->sequence == seq + 1);
    assert(fabs(frame->measurement.channel[0].frequency_hz - 3000) < 90);
    Scope_SetMode(SCOPE_AUTO, now_ms);
    Scope_SetTime(25);
    run_scope(300, 20000, 1);
    assert(frame->triggered && frame->count == 52);
    assert(fabs(frame->measurement.channel[0].frequency_hz - 20000) < 50);
    Scope_SetEdge(SCOPE_FALLING);
    unsigned relock_at = now_ms;
    do {
        run_scope(5, 5000, 1);
    } while (now_ms - relock_at < 1000 &&
             (!frame->triggered || fabs(frame->measurement.channel[0].frequency_hz - 5000) > 150));
    assert(now_ms - relock_at < 1000 && frame->triggered);
    trigger = (frame->count - 2) / 4U;
    a = Scope_FrameCode(frame, trigger, 0);
    b = Scope_FrameCode(frame, trigger + 1, 0);
    assert(a >= 2047 && b <= 2048);
    Scope_SetAutoRange(true);
    for (unsigned tenth = 1; tenth <= 30; ++tenth) {
        run_scope(600, 1000, tenth * 0.1);
        float occupy = frame->measurement.channel[0].peak_to_peak_mv /
                      (SCOPE_VERTICAL_DIVS * Scope_Settings()->vertical_mv_div);
        assert(occupy >= 0.60f && occupy <= 0.80f);
    }
    puts("DSP PASS: 100 Hz/20 kHz trigger, falling-edge relock within 1 s, normal hold, updated readings on single capture after a frequency change, 0.1-3.0 V auto range");
}

/* 核对最长冻结帧中两个通道的每个原始码。 */
static void verify_frame_storage(void)
{
    Scope_Init();
    assert(Scope_SetTime(SCOPE_MAX_US_DIV));
    Scope_SetTrigger(0);
    for (unsigned i = 0; i < SCOPE_HISTORY_SIZE; ++i) {
        unsigned a = (37U * i) & 4095U, b = (157U * i + 4095U) & 4095U;
        history[(end + i) & (SCOPE_HISTORY_SIZE - 1U)] = a | (b << 16U);
    }
    end += SCOPE_HISTORY_SIZE;
    available = SCOPE_HISTORY_SIZE;
    now_ms += 1000;
    Scope_Poll(now_ms);
    Scope_Stop();
    const ScopeFrame *frame = Scope_Frame();
    assert(frame->count == SCOPE_FRAME_CAPACITY);
    uint32_t first = end - frame->count;
    for (unsigned i = 0; i < frame->count; ++i) {
        uint32_t expected = Acquisition_Sample(first + i);
        for (unsigned ch = 0; ch < 2; ++ch) {
            assert(Scope_FrameCode(frame, i, ch) == Acquisition_Code(expected, ch));
        }
    }
    unsigned last = frame->count - 1U;
    for (unsigned ch = 0; ch < 2; ++ch) {
        float a = Acquisition_Code(Acquisition_Sample(first + last - 1U), ch);
        float b = Acquisition_Code(Acquisition_Sample(first + last), ch);
        assert(Scope_Interpolate(frame, last - 0.5f, ch) == 0.5f * (a + b));
        assert(Scope_Interpolate(frame, last, ch) == b);
    }
    puts("DSP PASS: longest 50 ms capture preserves both 12-bit channels and the final interpolation endpoint");
}

/* Confirm both channels retain their own voltage endpoints and quantization error bound. */
static void verify_stream_encoding(void)
{
    enum { POINTS = 256 };
    uint16_t input[POINTS * 2];
    uint8_t coded[(POINTS + STREAM_RANGE_BYTES) * 2];
    for (unsigned i = 0; i < POINTS; ++i) {
        input[2 * i] = (uint16_t)(1800 + i / 2U);
        input[2 * i + 1] = (uint16_t)(32 + 14U * (255U - i));
    }
    assert(Stream_Pack(coded, input, POINTS, 2) == sizeof coded);
    for (unsigned ch = 0; ch < 2; ++ch) {
        const uint8_t *range = coded + STREAM_RANGE_BYTES * ch;
        unsigned low = range[0] + (range[1] << 8);
        unsigned span = range[2] + (range[3] << 8);
        assert(low == (ch ? 32 : 1800) && span == (ch ? 3570 : 127));
        for (unsigned i = 0; i < POINTS; ++i) {
            unsigned index = i * 2 + ch;
            double restored = low + span * coded[2 * STREAM_RANGE_BYTES + index] / 255.0;
            assert(fabs(input[index] - restored) <= span / 510.0 + 0.001);
        }
    }
    for (unsigned i = 0; i < POINTS; ++i) input[i] = 2442;
    assert(Stream_Pack(coded, input, POINTS, 1) == POINTS + STREAM_RANGE_BYTES);
    assert(coded[0] + (coded[1] << 8) == 2442 && coded[2] == 0 && coded[3] == 0);
    for (unsigned i = 0; i < POINTS; ++i) assert(coded[STREAM_RANGE_BYTES + i] == 0);
    puts("DSP PASS: independent adaptive coding ranges, bounded voltage error, and exact DC");
}

int main(void)
{
    verify_measurement();
    verify_phase_angles();
    verify_fft();
    verify_scope();
    verify_frame_storage();
    verify_stream_encoding();
}
