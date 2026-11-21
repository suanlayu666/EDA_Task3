#include "spectrum.h"
#include "acquisition.h"
#include <math.h>

enum { FFT_N = SPECTRUM_POINTS, FFT_BITS = 12, FFT_BATCH = 256 };
static const float tau = 6.28318530718f;
static float real[FFT_N], imag[FFT_N];
static SampleReader read_sample;
static uint32_t first_sample;
static unsigned input_index, length, base, offset, fs;
static float scale, frequency, dc_code;
static float step_r, step_i, turn_r, turn_i;
static bool working, transforming;

/* 准备本级 FFT 蝶形或窗函数的旋转量。 */
static void reset_twiddle(float radians)
{
    step_r = cosf(radians);
    step_i = sinf(radians);
    turn_r = 1;
    turn_i = 0;
}

/* 推进旋转量，避免对每一点调用三角函数。 */
static void advance_twiddle(void)
{
    float next = turn_r * step_r - turn_i * step_i;
    turn_i = turn_r * step_i + turn_i * step_r;
    turn_r = next;
}

/* 计算 FFT 输入样本的位反转地址。 */
static unsigned reverse_bits(unsigned value)
{
#if defined(__arm__)
    unsigned reversed;
    __asm__("rbit %0, %1" : "=r"(reversed) : "r"(value));
    return reversed >> (32 - FFT_BITS);
#else
    unsigned reversed = 0;
    for (unsigned i = 0; i < FFT_BITS; ++i) {
        reversed = (reversed << 1) | (value & 1U);
        value >>= 1;
    }
    return reversed;
#endif
}

/* 为当前数据窗准备去直流、加窗与 FFT。 */
void Spectrum_Start(SampleReader sample, uint32_t first, unsigned sample_hz,
                    float mv_per_code, const Measurement *reference)
{
    if (working || reference->frequency_hz < 500.0f ||
        reference->frequency_hz * 5.0f >= 0.48f * sample_hz) return;
    read_sample = sample;
    first_sample = first;
    input_index = 0;
    fs = sample_hz;
    scale = mv_per_code;
    frequency = reference->frequency_hz;
    dc_code = reference->dc_mv / scale;
    working = true;
    transforming = false;
    reset_twiddle(tau / FFT_N);
}

/* 读取一个频点的平方幅值。 */
static float power_at(unsigned bin)
{
    return real[bin] * real[bin] + imag[bin] * imag[bin];
}

/* 按基波定位五个谐波并计算总谐波失真。 */
static Spectrum calculate_harmonics(void)
{
    Spectrum s = {0};
    for (unsigned i = 0; i < 5; ++i) {
        unsigned k = (unsigned)(frequency * (i + 1U) * FFT_N / fs + 0.5f);
        if (power_at(k - 1) > power_at(k)) --k;
        else if (power_at(k + 1) > power_at(k)) ++k;
        float left = logf(fmaxf(power_at(k - 1), 1e-12f));
        float center = logf(fmaxf(power_at(k), 1e-12f));
        float right = logf(fmaxf(power_at(k + 1), 1e-12f));
        float curvature = left - 2.0f * center + right;
        float shift = curvature < -0.0001f ? 0.5f * (left - right) / curvature : 0;
        shift = fmaxf(-0.5f, fminf(0.5f, shift));
        float peak_power = expf(center - 0.25f * (left - right) * shift);
        s.harmonic_mv[i] = sqrtf(peak_power) * (4.0f / FFT_N) * scale;
    }
    float sum = 0;
    for (unsigned i = 1; i < 5; ++i) sum += s.harmonic_mv[i] * s.harmonic_mv[i];
    s.valid = s.harmonic_mv[0] >= 5.0f;
    if (s.valid) s.thd_percent = 100.0f * sqrtf(sum) / s.harmonic_mv[0];
    return s;
}

/* 执行固定数量的 FFT 操作，完成后发布结果。 */
bool Spectrum_Poll(Spectrum *result)
{
    if (!working) return false;
    if (!transforming) {
        for (unsigned i = 0; i < FFT_BATCH; ++i) {
            unsigned j = reverse_bits(input_index);
            float code = Acquisition_Code(read_sample(first_sample + input_index), 0);
            real[j] = (code - dc_code) * (0.5f - 0.5f * turn_r);
            imag[j] = 0;
            ++input_index;
            advance_twiddle();
        }
        if (input_index == FFT_N) {
            transforming = true;
            length = 2;
            base = offset = 0;
            reset_twiddle(-tau / length);
        }
        return false;
    }

    for (unsigned i = 0; i < FFT_BATCH; ++i) {
        unsigned half = length / 2U;
        unsigned a = base + offset, b = a + half;
        float product_r = real[b] * turn_r - imag[b] * turn_i;
        float product_i = real[b] * turn_i + imag[b] * turn_r;
        real[b] = real[a] - product_r;
        imag[b] = imag[a] - product_i;
        real[a] += product_r;
        imag[a] += product_i;
        if (++offset < half) {
            advance_twiddle();
            continue;
        }
        offset = 0;
        turn_r = 1;
        turn_i = 0;
        base += length;
        if (base < FFT_N) continue;
        base = 0;
        length *= 2U;
        if (length > FFT_N) {
            *result = calculate_harmonics();
            working = false;
            return true;
        }
        reset_twiddle(-tau / length);
    }
    return false;
}

/* 判断 FFT 是否已开始且尚未完成。 */
bool Spectrum_Busy(void) { return working; }

/* 放弃已经失效的频谱任务。 */
void Spectrum_Cancel(void) { working = false; }
