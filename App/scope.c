#include "scope.h"
#include <math.h>

static const unsigned time_divs[] = {25, 50, 100, 200, 500, 1000, 2000, 5000};
static const unsigned volt_divs[] = {50, 100, 200, 250, 500, 1000};

static ScopeSettings settings;
static ScopeFrame frame;
static MeasurementSet recent;
static Spectrum spectrum;
static uint32_t last_frame_ms, last_measure_ms, mode_start_ms;
static uint32_t edge_index;
static float edge_fraction;
static unsigned previous, range_unstable;
static bool have_previous, trigger_armed, have_edge, spectrum_current;
static uint32_t spectrum_sequence;
static enum { SINGLE_WAIT, SINGLE_MEASURE, SINGLE_SPECTRUM } single_stage;

/* 按每格时间算出整屏所需的样本数。 */
static unsigned window_samples(void)
{
    return (uint64_t)SCOPE_SAMPLE_HZ * settings.time_us_div * SCOPE_HORIZONTAL_DIVS / 1000000U;
}

/* 清除上一次触发及边沿检测状态。 */
static void clear_trigger(void)
{
    have_previous = false;
    trigger_armed = false;
    have_edge = false;
    if (single_stage != SINGLE_WAIT) {
        Measurement_Cancel();
        Spectrum_Cancel();
        single_stage = SINGLE_WAIT;
    }
}

/* 设置上电默认的时基、增益、触发阈值和模式。 */
void Scope_Init(void)
{
    settings = (ScopeSettings){
        .time_us_div = 200,
        .vertical_mv_div = 500,
        .trigger_mv = 1500,
        .reference_mv = SCOPE_ADC_REFERENCE_MV,
        .mode = SCOPE_AUTO,
        .edge = SCOPE_RISING,
        .state = SCOPE_RUNNING,
    };
}

/* 提供触摸和串口共用的只读配置。 */
const ScopeSettings *Scope_Settings(void) { return &settings; }
/* 提供最近一次已发布的完整捕获。 */
const ScopeFrame *Scope_Frame(void) { return &frame; }

/* 返回当前触发模式的显示名称。 */
const char *Scope_ModeName(void)
{
    static const char *names[] = {"AUTO", "NORM", "SINGLE"};
    return names[settings.mode];
}

/* 返回运行、武装或冻结的显示名称。 */
const char *Scope_StateName(void)
{
    static const char *names[] = {"RUN", "ARM", "STOP"};
    return names[settings.state];
}

/* 验证时基档位，修改取帧窗口。 */
bool Scope_SetTime(unsigned us_div)
{
    if (settings.state == SCOPE_FROZEN) return false;
    for (unsigned i = 0; i < sizeof time_divs / sizeof time_divs[0]; ++i) {
        if (us_div == time_divs[i]) {
            settings.time_us_div = us_div;
            clear_trigger();
            return true;
        }
    }
    return false;
}

/* 应用手动数字增益，并退出自动量程。 */
bool Scope_SetVertical(unsigned mv_div)
{
    if (mv_div < 10 || mv_div > 2000) return false;
    settings.vertical_mv_div = mv_div;
    settings.auto_range = false;
    range_unstable = 0;
    return true;
}

/* 按输入电压范围更新触发阈值。 */
bool Scope_SetTrigger(unsigned mv)
{
    if (mv > settings.reference_mv) return false;
    if (settings.trigger_mv != mv) {
        settings.trigger_mv = mv;
        clear_trigger();
    }
    return true;
}

/* 应用经实测的 ADC 参考电压，刷新测量。 */
bool Scope_SetReference(unsigned mv)
{
    if (mv < 3000 || mv > 3600 || mv < settings.trigger_mv ||
        settings.state == SCOPE_FROZEN) return false;
    settings.reference_mv = mv;
    Measurement_Cancel();
    Spectrum_Cancel();
    recent = (MeasurementSet){0};
    spectrum_current = false;
    clear_trigger();
    return true;
}

/* 切换模式并立即进入其运行或武装状态。 */
void Scope_SetMode(TriggerMode mode, uint32_t now)
{
    settings.mode = mode;
    Scope_Run(now);
}

/* 更换穿越方向并重置检测历史。 */
void Scope_SetEdge(TriggerEdge edge)
{
    if (settings.edge != edge) {
        settings.edge = edge;
        clear_trigger();
    }
}

/* 切换屏幕和查询是否包含第二路。 */
void Scope_SetDual(bool dual) { settings.dual = dual; }

/* 启停自动量程并清除换挡计数。 */
void Scope_SetAutoRange(bool auto_range)
{
    settings.auto_range = auto_range;
    range_unstable = 0;
}

/* 恢复取帧，或重新武装单次模式。 */
void Scope_Run(uint32_t now)
{
    settings.state = settings.mode == SCOPE_SINGLE ? SCOPE_ARMED : SCOPE_RUNNING;
    mode_start_ms = now;
    clear_trigger();
}

/* 冻结最近捕获，保持底层采样运行。 */
void Scope_Stop(void)
{
    settings.state = SCOPE_FROZEN;
    clear_trigger();
}

/* 由触摸切到下一档时基。 */
void Scope_CycleTime(void)
{
    unsigned count = sizeof time_divs / sizeof time_divs[0], i = 0;
    while (i < count && time_divs[i] <= settings.time_us_div) ++i;
    Scope_SetTime(time_divs[i % count]);
}

/* 由触摸切到下一档手动垂直增益。 */
void Scope_CycleVertical(void)
{
    unsigned count = sizeof volt_divs / sizeof volt_divs[0], i = 0;
    while (i < count && volt_divs[i] <= settings.vertical_mv_div) ++i;
    Scope_SetVertical(volt_divs[i % count]);
}

/* 检测每个样本的阈值穿越，保存带小数位置的触发点。 */
void Scope_Consume(const SampleBlock *block)
{
    if (block->discontinuity) clear_trigger();
    if (settings.state == SCOPE_FROZEN) return;
    const int trigger = (settings.trigger_mv * 4095U + settings.reference_mv / 2U) /
                        settings.reference_mv;
    const bool rising = settings.edge == SCOPE_RISING;
    const unsigned window = window_samples();

    for (unsigned i = 0; i < block->count; ++i) {
        const int sample = Acquisition_Code(block->data[i], 0);
        const int before = (int)previous;
        uint32_t index = block->first + i;
        if (rising ? sample <= trigger - 5 : sample >= trigger + 5) trigger_armed = true;
        if (have_previous && trigger_armed &&
            (rising ? before < trigger && sample >= trigger : before > trigger && sample <= trigger)) {
            /* Keep the first crossing until its post-trigger samples are ready. */
            if (!have_edge && Acquisition_Count() - (Acquisition_End() - index) >= window / 4U + 1U) {
                edge_index = index - 1U;
                edge_fraction = (float)(trigger - before) / (sample - before);
                have_edge = true;
            }
            trigger_armed = false;
        }
        previous = (unsigned)sample;
        have_previous = true;
    }
}

/* 连续超出目标占比时选择最接近 70% 的档位。 */
static void update_range(void)
{
    if (!settings.auto_range || settings.state == SCOPE_FROZEN) return;
    float peak_span = recent.channel[0].peak_to_peak_mv;
    if (settings.dual) peak_span = fmaxf(peak_span, recent.channel[1].peak_to_peak_mv);
    if (peak_span < 75) {
        range_unstable = 0;
        return;
    }
    float occupied = peak_span / (SCOPE_VERTICAL_DIVS * settings.vertical_mv_div);
    if (occupied >= 0.61f && occupied <= 0.79f) {
        range_unstable = 0;
    } else if (++range_unstable >= 3U) {
        settings.vertical_mv_div = (unsigned)(peak_span / (0.70f * SCOPE_VERTICAL_DIVS) + 0.5f);
        range_unstable = 0;
    }
}

/* 保存新帧所有样本、刻度与测量快照。 */
static void publish_frame(uint32_t first, unsigned count, float fraction, bool triggered, uint32_t now)
{
    for (unsigned i = 0; i < count; ++i) {
        uint32_t sample = Acquisition_Sample(first + i);
        uint16_t a = Acquisition_Code(sample, 0), b = Acquisition_Code(sample, 1);
        frame.samples[i][0] = (uint8_t)a;
        frame.samples[i][1] = (uint8_t)((a >> 8U) | (b << 4U));
        frame.samples[i][2] = (uint8_t)(b >> 4U);
    }
    frame.count = count;
    frame.reference_mv = settings.reference_mv;
    frame.time_us_div = settings.time_us_div;
    frame.start_fraction = fraction;
    frame.triggered = triggered;
    frame.measurement = recent;
    frame.spectrum = spectrum_current ? spectrum : (Spectrum){0};
    ++frame.sequence;
    frame.captured_at_ms = last_frame_ms = now;
}

/* 保留一次有效触发，用触发后的样本测量并发布完整单次帧。 */
static void poll_single(uint32_t now, uint32_t end, unsigned window)
{
    unsigned post_trigger = window - window / 4U + 2U;
    float scale = settings.reference_mv / 4095.0f;
    if (!have_edge || end - edge_index < post_trigger ||
        end - edge_index < SCOPE_MEASURE_SAMPLES) return;

    if (single_stage == SINGLE_WAIT) {
        Spectrum_Cancel();
        Measurement_Start(Acquisition_Sample, edge_index, SCOPE_MEASURE_SAMPLES,
                          SCOPE_SAMPLE_HZ, scale);
        single_stage = SINGLE_MEASURE;
        return;
    }
    if (single_stage == SINGLE_MEASURE) {
        MeasurementSet result;
        if (!Measurement_Poll(&result)) return;
        result.sequence = recent.sequence + 1U;
        recent = result;
        spectrum_current = false;
        Spectrum_Start(Acquisition_Sample, edge_index, SCOPE_SAMPLE_HZ, scale,
                       &recent.channel[0]);
        single_stage = SINGLE_SPECTRUM;
    }
    if (Spectrum_Busy()) {
        if (!Spectrum_Poll(&spectrum)) return;
        spectrum_current = true;
    }
    publish_frame(edge_index - window / 4U, window + 2U, edge_fraction, true, now);
    Scope_Stop();
}

/* 推进分析任务；按触发模式和时间条件发布捕获。 */
void Scope_Poll(uint32_t now)
{
    uint32_t end = Acquisition_End();
    unsigned available = Acquisition_Count();
    unsigned window = window_samples();
    if (have_edge && (available < window / 4U || end - edge_index > available - window / 4U)) {
        clear_trigger();
    }
    if (settings.state == SCOPE_ARMED) {
        poll_single(now, end, window);
        return;
    }
    if (settings.state == SCOPE_FROZEN) {
        MeasurementSet unused;
        Measurement_Poll(&unused);
        Spectrum_Poll(&spectrum);
        return;
    }
    if (available >= SCOPE_MEASURE_SAMPLES && !Measurement_Busy() &&
        (recent.sequence == 0 || now - last_measure_ms >= SCOPE_MEASURE_INTERVAL_MS)) {
        Measurement_Start(Acquisition_Sample, end - SCOPE_MEASURE_SAMPLES,
                          SCOPE_MEASURE_SAMPLES, SCOPE_SAMPLE_HZ, settings.reference_mv / 4095.0f);
        last_measure_ms = now;
    }

    MeasurementSet result;
    if (Measurement_Poll(&result)) {
        result.sequence = recent.sequence + 1U;
        recent = result;
        update_range();
        const Measurement *ch1 = &recent.channel[0];
        if (ch1->frequency_hz < 500 || ch1->frequency_hz * 5 >= 0.48f * SCOPE_SAMPLE_HZ) {
            spectrum_current = false;
        } else if (!Spectrum_Busy()) {
            uint32_t first = result.sample_start + SCOPE_MEASURE_SAMPLES - SPECTRUM_POINTS;
            Spectrum_Start(Acquisition_Sample, first, SCOPE_SAMPLE_HZ,
                           settings.reference_mv / 4095.0f, ch1);
            spectrum_sequence = recent.sequence;
        }
    }
    if (Spectrum_Poll(&spectrum)) spectrum_current = spectrum_sequence == recent.sequence;

    if (now - last_frame_ms < SCOPE_FRAME_INTERVAL_MS || available < window + 2U) return;
    if (have_edge && end - edge_index >= window - window / 4U + 2U) {
        publish_frame(edge_index - window / 4U, window + 2U, edge_fraction, true, now);
        have_edge = false;
    } else if (!have_edge && settings.mode == SCOPE_AUTO &&
               now - mode_start_ms >= SCOPE_AUTO_TIMEOUT_MS &&
               (frame.sequence == 0 || !frame.triggered || now - last_frame_ms >= SCOPE_AUTO_TIMEOUT_MS)) {
        publish_frame(end - window - 2U, window + 2U, 0, false, now);
    }
}
