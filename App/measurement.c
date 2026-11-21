#include "measurement.h"
#include "acquisition.h"
#include <math.h>
#include <string.h>

enum { BATCH_POINTS = 256 };
static const float tau = 6.28318530718f;

typedef struct {
    unsigned low, high, previous, rails, edges, hysteresis;
    uint32_t sum, edge_start_sum, edge_end_sum;
    float mid, min_period, max_period;
    unsigned first_whole, last_whole;
    bool armed;
} ChannelStats;

/* 保存一轮分批执行的测量任务。 */
static struct {
    SampleReader read;
    uint32_t first;
    unsigned count, fs, at, stage;
    float scale;
    ChannelStats ch[2];
    MeasurementSet result;
    unsigned phase_stop;
    float phase_cos, phase_sin, step_cos, step_sin, re[2], im[2];
} job;

/* 初始化一轮测量，绑定连续的历史样本。 */
void Measurement_Start(SampleReader read, uint32_t first, unsigned count,
                       unsigned sample_hz, float mv_per_code)
{
    memset(&job, 0, sizeof job);
    job.read = read;
    job.first = first;
    job.count = count;
    job.fs = sample_hz;
    job.scale = mv_per_code;
    job.result.sample_start = first;
    job.ch[0].low = job.ch[1].low = 4095;
    job.ch[0].min_period = job.ch[1].min_period = (float)count;
    job.stage = count >= 2 ? 1U : 0U;
}

/* 判断当前测量是否还在进行。 */
bool Measurement_Busy(void) { return job.stage != 0; }

/* 放弃已经失效的输入测量。 */
void Measurement_Cancel(void) { job.stage = 0; }

/* 扫描上下峰值，确定幅度和穿越阈值。 */
static void find_extrema(unsigned stop)
{
    for (; job.at < stop; ++job.at) {
        uint32_t sample = job.read(job.first + job.at);
        for (unsigned ch = 0; ch < 2; ++ch) {
            unsigned code = Acquisition_Code(sample, ch);
            ChannelStats *v = &job.ch[ch];
            if (code < v->low) v->low = code;
            if (code > v->high) v->high = code;
        }
    }
    if (job.at != job.count) return;
    for (unsigned ch = 0; ch < 2; ++ch) {
        ChannelStats *v = &job.ch[ch];
        v->mid = (v->low + v->high) * 0.5f;
        v->hysteresis = (v->high - v->low) / 20U + 3U;
    }
    job.at = 0;
    job.stage = 2;
}

/* 插值计算周期，按峰值附近停留比例识别波形。 */
static void count_edges(unsigned stop)
{
    for (; job.at < stop; ++job.at) {
        uint32_t sample = job.read(job.first + job.at);
        for (unsigned ch = 0; ch < 2; ++ch) {
            unsigned code = Acquisition_Code(sample, ch);
            ChannelStats *v = &job.ch[ch];
            Measurement *m = &job.result.channel[ch];
            unsigned span = v->high - v->low;
            if ((code - v->low) * 8U <= span || (v->high - code) * 8U <= span) ++v->rails;
            if (code <= v->mid - v->hysteresis) v->armed = true;
            if (job.at && v->armed && v->previous < v->mid && code >= v->mid) {
                float edge = (job.at - 1U) + (v->mid - v->previous) / (code - v->previous);
                if (v->edges++ == 0) {
                    m->first_edge = edge;
                    v->first_whole = job.at;
                    v->edge_start_sum = v->sum;
                } else {
                    float period = edge - m->last_edge;
                    v->min_period = fminf(v->min_period, period);
                    v->max_period = fmaxf(v->max_period, period);
                }
                m->last_edge = edge;
                v->last_whole = job.at;
                v->edge_end_sum = v->sum;
                v->armed = false;
            }
            v->sum += code;
            v->previous = code;
        }
    }
    if (job.at != job.count) return;

    for (unsigned ch = 0; ch < 2; ++ch) {
        ChannelStats *v = &job.ch[ch];
        Measurement *m = &job.result.channel[ch];
        unsigned span = v->high - v->low;
        float period_span = m->last_edge - m->first_edge;
        m->dc_mv = (float)v->sum / job.count * job.scale;
        m->peak_to_peak_mv = span * job.scale;
        m->center_mv = v->mid * job.scale;
        m->type = span < 25 ? WAVE_DC : WAVE_UNKNOWN;
        if (v->edges < 2 || span < 12 || period_span < 4 * (v->edges - 1U)) continue;
        float mean_period = period_span / (v->edges - 1U);
        if (v->max_period - v->min_period > fmaxf(3.0f, mean_period * 0.18f)) continue;
        m->periods = v->edges - 1U;
        m->frequency_hz = (float)job.fs * m->periods / period_span;
        m->dc_mv = (float)(v->edge_end_sum - v->edge_start_sum) /
                   (v->last_whole - v->first_whole) * job.scale;
        float dwell = (float)v->rails / job.count;
        m->type = dwell > 0.72f ? WAVE_SQUARE : dwell > 0.35f ? WAVE_SINE :
                  dwell > 0.13f ? WAVE_TRIANGLE : WAVE_UNKNOWN;
    }
    job.stage = 3;
}

/* 两路同频时，建立共用的相位参考。 */
static bool prepare_phase(void)
{
    const Measurement *a = &job.result.channel[0], *b = &job.result.channel[1];
    if (a->frequency_hz < 80 || fabsf(a->frequency_hz - b->frequency_hz) > a->frequency_hz * 0.015f) {
        return false;
    }
    float step = tau * a->frequency_hz / job.fs;
    job.step_cos = cosf(step);
    job.step_sin = sinf(step);
    job.phase_cos = 1;
    job.at = (unsigned)a->first_edge;
    job.phase_stop = (unsigned)a->last_edge;
    job.stage = 4;
    return true;
}

/* 用同一个基波参考计算 CH2 相对 CH1 的相位。 */
static void measure_phase(void)
{
    float dc_a = job.result.channel[0].dc_mv / job.scale;
    float dc_b = job.result.channel[1].dc_mv / job.scale;
    for (unsigned n = 0; n < BATCH_POINTS && job.at < job.phase_stop; ++n) {
        uint32_t sample = job.read(job.first + job.at);
        const float a = Acquisition_Code(sample, 0) - dc_a;
        const float b = Acquisition_Code(sample, 1) - dc_b;
        job.re[0] += a * job.phase_cos;
        job.im[0] -= a * job.phase_sin;
        job.re[1] += b * job.phase_cos;
        job.im[1] -= b * job.phase_sin;
        float next = job.phase_cos * job.step_cos - job.phase_sin * job.step_sin;
        job.phase_sin = job.phase_sin * job.step_cos + job.phase_cos * job.step_sin;
        job.phase_cos = next;
        ++job.at;
    }
    if (job.at < job.phase_stop) return;
    float in_phase = job.re[0] * job.re[1] + job.im[0] * job.im[1];
    float quadrature = job.re[0] * job.im[1] - job.im[0] * job.re[1];
    float phase = atan2f(quadrature, in_phase) * (360.0f / tau);
    job.result.phase_deg = phase < 0 ? phase + 360.0f : phase;
    job.result.phase_valid = true;
    job.stage = 5;
}

/* 推进一批计算，完成后发布整组测量。 */
bool Measurement_Poll(MeasurementSet *result)
{
    unsigned stop = job.at + BATCH_POINTS;
    if (stop > job.count) stop = job.count;
    switch (job.stage) {
    case 1: find_extrema(stop); return false;
    case 2: count_edges(stop); return false;
    case 3: if (prepare_phase()) return false; break;
    case 4: measure_phase(); if (job.stage != 5) return false; break;
    default: return false;
    }
    *result = job.result;
    job.stage = 0;
    return true;
}

/* 将识别结果转换成 LCD 和串口共用的名称。 */
const char *Measurement_Name(WaveType type)
{
    static const char *names[] = {"DC", "SINE", "SQUARE", "TRIANGLE", "UNKNOWN"};
    return names[type];
}
