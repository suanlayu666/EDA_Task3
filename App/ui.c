#include "ui.h"
#include "scope.h"
#include "lcd.h"
#include "touch.h"
#include "screen_delay.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

enum {
    PLOT_X = 24, PLOT_Y = 69, PLOT_W = 211, PLOT_H = 151, STRIPE_ROWS = 8,
    BUTTON_Y = 246, BUTTON_W = 60, BUTTON_H = 24, TOUCH_MS = 16,
    SCAN_AFTER = 16, SCAN_BEFORE = 40, SCAN_FRESH_GUARD = 16,
    SCAN_START_BAND = 48, SCAN_MAX_LINE = 400,
    SCAN_WRAP_DELTA = 100, SCAN_TIMEOUT_MS = 35,
    CAL_LEFT = 22, CAL_TOP = 39, CAL_RIGHT = 217, CAL_BOTTOM = 294,
};

enum {
    BG = 0x0841, GRID = 0x3186, BORDER = 0x6b6d, TEXT = 0xe71c,
    CH1 = 0x07ea, CH2 = 0x37bf, TRIGGER = 0xf9e0, CURSOR = 0xf81f,
};

typedef struct {
    int lo[2][PLOT_W], hi[2][PLOT_W];
    bool spectrum, dual, frozen, triggered;
    unsigned time_div, requested_time, voltage_div, trigger_mv, fps;
    bool waiting;
    int trigger_y, trigger_x, cursor[2];
    int bars[5];
    float center_mv;
    MeasurementSet measurement;
    Spectrum harmonics;
    char mode[7], state[6];
    TriggerEdge edge;
} Plot;

static Plot plot;
static uint16_t stripe[STRIPE_ROWS * PLOT_W];
static uint16_t grid_row[PLOT_W];
extern const unsigned char asc2_1206[95][12];
static unsigned plot_row, footer_row;
static unsigned fps, frame_counter;
static uint32_t fps_start, next_display, touch_at;
static struct {
    unsigned line, cycle, drawing_cycle;
    uint32_t read_at, changed_at, misses;
    bool synchronized, late;
} panel_scan;
static unsigned cursor_pos[2] = {58, 151};
static int drag_cursor = -1;
static bool can_draw, have_touch, was_pressed, want_spectrum, set_level;
static bool calibrating, draw_calibration, repaint, axis_spectrum;
static unsigned cal_point;
static uint16_t cal_samples[4][2];
static const char *const buttons[] = {
    "RUN/STOP", "MODE", "TIME", "V/DIV",
    "EDGE", "LEVEL", "SINGLE", "CH1/2",
    "RANGE", "FFT", "CURSORS", "CAL"
};

/* 按固定宽度画文字并覆盖残留字符。 */
static void label(unsigned x, unsigned y, unsigned width, uint16_t color, const char *text)
{
    unsigned chars = 0;
    while (chars < width && text[chars]) ++chars;
    if (x + width * 6U > lcddev.width || y + 12U > lcddev.height) return;
    LCD_Set_Window(x, y, width * 6U, 12);
    LCD_WriteRAM_Prepare();
    for (unsigned row = 0; row < 12U; ++row) {
        unsigned offset = row / 8U, mask = 0x80U >> (row % 8U);
        for (unsigned i = 0; i < width; ++i) {
            unsigned ch = i < chars ? (unsigned char)text[i] : ' ';
            if (ch < ' ' || ch > '~') ch = '?';
            const unsigned char *glyph = asc2_1206[ch - ' '];
            for (unsigned column = 0; column < 6U; ++column) {
                LCD_WriteRAM(glyph[column * 2U + offset] & mask ? color : BG);
            }
        }
    }
    LCD_Set_Window(0, 0, lcddev.width, lcddev.height);
}

/* 把电压换算成绘图区的像素行。 */
static int voltage_to_row(float mv, float center, unsigned mv_per_div)
{
    return (int)lroundf((PLOT_H - 1) * 0.5f -
                      (mv - center) * (PLOT_H - 1) / (SCOPE_VERTICAL_DIVS * mv_per_div));
}

/* 把像素位置限制在绘图区内。 */
static int clamp_row(int y)
{
    return y < 0 ? 0 : y >= PLOT_H ? PLOT_H - 1 : y;
}

/* 保存每一像素列覆盖样本的上下极值。 */
static void prepare_trace(const ScopeFrame *frame, unsigned channel, float scale)
{
    const float step = (frame->count - 2U) / (float)(PLOT_W - 1);
    for (unsigned x = 0; x < PLOT_W; ++x) {
        float start = frame->start_fraction + x * step;
        float stop = x + 1U < PLOT_W ? start + step : start;
        /* 限制最后一列的取样区间，避免浮点舍入越过帧尾。 */
        stop = fminf(stop, frame->count - 2U + frame->start_fraction);
        float a = Scope_Interpolate(frame, start, channel);
        float b = Scope_Interpolate(frame, stop, channel);
        float low = fminf(a, b), high = fmaxf(a, b);
        unsigned lowest_code = 4095U, highest_code = 0;
        for (unsigned i = (unsigned)start + 1U; i <= (unsigned)stop; ++i) {
            unsigned code = Scope_FrameCode(frame, i, channel);
            if (code < lowest_code) lowest_code = code;
            if (code > highest_code) highest_code = code;
        }
        if (lowest_code < low) low = lowest_code;
        if (highest_code > high) high = highest_code;
        plot.lo[channel][x] = clamp_row(voltage_to_row(high * scale, plot.center_mv, plot.voltage_div));
        plot.hi[channel][x] = clamp_row(voltage_to_row(low * scale, plot.center_mv, plot.voltage_div));
    }
}

/* 将五次谐波的幅值换算成柱状图高度。 */
static void prepare_bars(void)
{
    float largest = 1.0f;
    for (unsigned i = 0; i < 5; ++i) largest = fmaxf(largest, plot.harmonics.harmonic_mv[i]);
    for (unsigned i = 0; i < 5; ++i) {
        plot.bars[i] = plot.harmonics.valid ?
            (int)(plot.harmonics.harmonic_mv[i] * (PLOT_H - 12.0f) / largest) : 0;
    }
}

/* 用串口共用的捕获和测量数据准备一帧画面。 */
static void prepare_frame(const ScopeFrame *frame, const ScopeSettings *settings, uint32_t now)
{
    plot.spectrum = want_spectrum;
    plot.dual = settings->dual;
    plot.frozen = settings->state == SCOPE_FROZEN;
    plot.triggered = frame->triggered;
    plot.waiting = !frame->triggered || (!plot.frozen && now - frame->captured_at_ms >= 180U);
    plot.time_div = frame->count ? frame->time_us_div : settings->time_us_div;
    plot.requested_time = settings->time_us_div;
    plot.voltage_div = settings->vertical_mv_div;
    plot.trigger_mv = settings->trigger_mv;
    plot.center_mv = frame->measurement.sequence ? frame->measurement.channel[0].center_mv : 1650.0f;
    plot.trigger_y = clamp_row(voltage_to_row(plot.trigger_mv, plot.center_mv, plot.voltage_div));
    plot.trigger_x = (PLOT_W - 1) / 4;
    plot.cursor[0] = (int)cursor_pos[0];
    plot.cursor[1] = (int)cursor_pos[1];
    plot.fps = fps;
    plot.measurement = frame->measurement;
    plot.harmonics = frame->spectrum;
    plot.edge = settings->edge;
    snprintf(plot.mode, sizeof plot.mode, "%s", Scope_ModeName());
    snprintf(plot.state, sizeof plot.state, "%s", Scope_StateName());
    if (plot.spectrum) {
        prepare_bars();
    } else if (frame->count >= 2U) {
        float scale = frame->reference_mv / 4095.0f;
        for (unsigned ch = 0; ch < (plot.dual ? 2U : 1U); ++ch) {
            prepare_trace(frame, ch, scale);
        }
    } else {
        for (unsigned ch = 0; ch < 2U; ++ch) {
            for (unsigned x = 0; x < PLOT_W; ++x) plot.lo[ch][x] = plot.hi[ch][x] = (PLOT_H - 1) / 2;
        }
    }
    plot_row = 0;
    footer_row = 0;
    panel_scan.late = false;
}

/* 读取 ILI9341 扫描行。第一个返回值为占位，高低字节随后返回。 */
static unsigned scanline(void)
{
    (void)LCD_ReadReg(0x45);
    unsigned high = LCD_RD_DATA() & 3U;
    return (high << 8U) | (LCD_RD_DATA() & 255U);
}

/* 每毫秒读一次扫描行，识别屏幕何时开始下一轮扫描。 */
static void read_panel_scan(uint32_t now)
{
    if (lcddev.id != 0x9341 || now == panel_scan.read_at) return;
    if (panel_scan.read_at == 0) panel_scan.changed_at = now;
    panel_scan.read_at = now;
    unsigned line = scanline();
    if (line <= SCAN_MAX_LINE && line != panel_scan.line) {
        if (panel_scan.line > line + SCAN_WRAP_DELTA) {
            ++panel_scan.cycle;
            panel_scan.synchronized = true;
        }
        panel_scan.line = line;
        panel_scan.changed_at = now;
    }
    if (now - panel_scan.changed_at > SCAN_TIMEOUT_MS) panel_scan.synchronized = false;
}

/* 等上一轮扫描过条带，再赶在下一轮扫描到该条带前写入。 */
static bool stripe_is_safe(unsigned row, unsigned count, uint32_t now)
{
    if (panel_scan.late) return true;
    if (!panel_scan.synchronized) {
        if (row == 0 && lcddev.id == 0x9341 && now - panel_scan.changed_at <= SCAN_TIMEOUT_MS) return false;
        panel_scan.late = true;
        return true;
    }
    unsigned top = PLOT_Y + row;
    unsigned limit = top + count + SCAN_AFTER;
    if (row == 0) {
        if (panel_scan.line < limit || panel_scan.line >= limit + SCAN_START_BAND) return false;
        panel_scan.drawing_cycle = panel_scan.cycle;
    }
    unsigned elapsed = panel_scan.cycle - panel_scan.drawing_cycle;
    if (elapsed == 0) return panel_scan.line >= limit;
    if (elapsed == 1) {
        if (panel_scan.line + SCAN_BEFORE < top) return true;
        /* 靠近写入期限时重新读扫描行，避免使用上一毫秒的读数。 */
        unsigned line = scanline();
        if (line >= panel_scan.line && line + SCAN_FRESH_GUARD < top) return true;
    }
    ++panel_scan.misses;
    panel_scan.late = true;
    return true;
}

/* 只给当前条带内的波形区间上色。 */
static void paint_trace(unsigned channel, int start, int end, uint16_t color)
{
    for (unsigned x = 0; x < PLOT_W; ++x) {
        int low = plot.lo[channel][x], high = plot.hi[channel][x];
        if (low < start) low = start;
        if (high >= end) high = end - 1;
        for (int y = low; y <= high; ++y) stripe[(y - start) * PLOT_W + x] = color;
    }
}

/* 把底色、参考线及波形或频谱合成条带，一次写进 LCD。 */
static void paint_stripe(unsigned row, unsigned count)
{
    for (unsigned j = 0; j < count; ++j) {
        int y = (int)(row + j);
        uint16_t *pixels = &stripe[j * PLOT_W];
        memcpy(pixels, grid_row, sizeof grid_row);
        if (y % 25 == 0) {
            uint16_t color = y == 0 || y == PLOT_H - 1 ? BORDER : GRID;
            for (int x = 1; x < PLOT_W - 1; ++x) pixels[x] = color;
        }
        if (plot.spectrum) {
            for (int i = 0; i < 5; ++i) {
                if (!plot.bars[i] || y < PLOT_H - 1 - plot.bars[i]) continue;
                for (int x = 42 * i + 8; x <= 42 * i + 32; ++x) pixels[x] = CH1;
            }
        } else if (y == plot.trigger_y) {
            for (int x = 0; x < PLOT_W; ++x) if ((x + y) & 4) pixels[x] = TRIGGER;
        } else if (plot.triggered && ((plot.trigger_x + y) & 4)) {
            pixels[plot.trigger_x] = TRIGGER;
        }
    }
    if (!plot.spectrum) {
        if (plot.dual) paint_trace(1, row, row + count, CH2);
        paint_trace(0, row, row + count, CH1);
        if (plot.frozen) {
            for (unsigned j = 0; j < count; ++j) {
                stripe[j * PLOT_W + plot.cursor[0]] = CURSOR;
                stripe[j * PLOT_W + plot.cursor[1]] = CURSOR;
            }
        }
    }
    LCD_Set_Window(PLOT_X, PLOT_Y + row, PLOT_W, count);
    LCD_Color_Fill(PLOT_X, PLOT_Y + row, PLOT_X + PLOT_W - 1, PLOT_Y + row + count - 1, stripe);
    LCD_Set_Window(0, 0, lcddev.width, lcddev.height);
}

/* 绘制一个触摸按钮及其选中状态。 */
static void paint_button(unsigned i)
{
    unsigned x = i % 4U * BUTTON_W, y = BUTTON_Y + i / 4U * BUTTON_H;
    POINT_COLOR = BORDER;
    LCD_DrawRectangle(x + 1, y + 1, x + BUTTON_W - 3, y + BUTTON_H - 3);
    uint16_t color = ((i == 5 && set_level) || (i == 8 && Scope_Settings()->auto_range) ||
                      (i == 9 && want_spectrum)) ? TRIGGER : TEXT;
    label(x + 5, y + 7, 8, color, buttons[i]);
}

/* 每轮只绘制一条读数或刻度，及时处理采样。 */
static void paint_readout(unsigned line)
{
    char text[64];
    if (line == 0) {
        snprintf(text, sizeof text, "%s %s %s %s %u FPS     ", plot.mode, plot.state,
                 plot.edge == SCOPE_RISING ? "RISE" : "FALL",
                 plot.waiting ? "WAIT" : "TRIG", plot.fps);
        label(4, 2, 38, TEXT, text);
    } else if (line == 1 || line == 2) {
        unsigned ch = line - 1;
        const Measurement *m = &plot.measurement.channel[ch];
        snprintf(text, sizeof text, "%u %.1fHz %.3fVpp %.3fVdc %s", ch + 1U,
                 m->frequency_hz, m->peak_to_peak_mv / 1000.0f,
                 m->dc_mv / 1000.0f, Measurement_Name(m->type));
        label(4, 17 + 13 * ch, 39, ch ? CH2 : CH1, (!ch || plot.dual) ? text : "");
    } else if (line == 3) {
        snprintf(text, sizeof text, "%uus/div %.3fV/div L%.2fV", plot.time_div,
                 plot.voltage_div / 1000.0f, plot.trigger_mv / 1000.0f);
        if (plot.requested_time != plot.time_div) {
            snprintf(text, sizeof text, "%uus/div %umV/div SET %uus",
                     plot.time_div, plot.voltage_div, plot.requested_time);
        }
        label(4, 44, 39, TEXT, plot.spectrum ? "CH1 HARMONICS 1-5 (Vpk)" : text);
    } else if (line == 4) {
        if (plot.spectrum && plot.harmonics.valid) {
            snprintf(text, sizeof text, "THD %.2f%%  A1 %.3fV", plot.harmonics.thd_percent,
                     plot.harmonics.harmonic_mv[0] / 1000.0f);
        } else if (plot.frozen && !plot.spectrum) {
            float delta = abs(plot.cursor[0] - plot.cursor[1]) * (10.0f * plot.time_div / (PLOT_W - 1U));
            snprintf(text, sizeof text, "dT %.1fus   1/dT %.1fHz", delta, delta ? 1e6f / delta : 0.0f);
        } else if (plot.dual && plot.measurement.phase_valid) {
            snprintf(text, sizeof text, "PHASE CH2-CH1 %.1f deg", plot.measurement.phase_deg);
        } else {
            snprintf(text, sizeof text, "%s", plot.spectrum ? "WAITING FOR PERIODIC INPUT" : "");
        }
        label(4, 56, 39, CURSOR, text);
    } else if (line <= 11) {
        unsigned axis = line - 5U;
        float mv = plot.center_mv + (3 - (int)axis) * (float)plot.voltage_div;
        snprintf(text, sizeof text, mv >= 0 && mv < 10000 ? "%4.2f" : "%4.1f", mv / 1000.0f);
        label(0, PLOT_Y - 4 + 25 * axis, 4, TEXT, plot.spectrum ? "" : text);
    } else if (line == 12) {
        if (axis_spectrum != plot.spectrum) {
            LCD_Fill(PLOT_X, 221, lcddev.width - 1, BUTTON_Y - 1, BG);
            axis_spectrum = plot.spectrum;
        }
        if (plot.spectrum) {
            for (unsigned i = 0; i < 5; ++i) {
                snprintf(text, sizeof text, "%.3f", plot.harmonics.harmonic_mv[i] / 1000.0f);
                label(PLOT_X + i * 42 + 6, 222, 5, CH1, plot.harmonics.valid ? text : "-");
                snprintf(text, sizeof text, "%u", i + 1U);
                label(PLOT_X + i * 42 + 18, 234, 3, TEXT, text);
            }
        } else {
            snprintf(text, sizeof text, "%ums", plot.time_div / 100U);
            label(24, 223, 34, TEXT, "");
            label(24, 232, 34, TEXT, "");
            label(24, 224, 8, TEXT, "0");
            if (plot.time_div < 100) snprintf(text, sizeof text, "%uus", 10U * plot.time_div);
            label(194, 224, 7, TEXT, text);
        }
    }
}

/* 进入四点校准并重置触摸状态。 */
void Ui_RequestCalibration(void)
{
    if (!can_draw) return;
    calibrating = draw_calibration = true;
    cal_point = 0;
    was_pressed = false;
}

/* 每次松手记录一个校准点，采样照常运行。 */
static void calibrate(uint32_t now)
{
    static const unsigned xs[4] = {CAL_LEFT, CAL_RIGHT, CAL_LEFT, CAL_RIGHT};
    static const unsigned ys[4] = {CAL_TOP, CAL_TOP, CAL_BOTTOM, CAL_BOTTOM};
    if (draw_calibration) {
        if (cal_point) {
            const unsigned x = xs[cal_point - 1U], y = ys[cal_point - 1U];
            LCD_Fill(x - 9, y - 9, x + 9, y + 9, BG);
        }
        label(42, 146, 28, TEXT, "Touch the yellow cross");
        const unsigned x = xs[cal_point], y = ys[cal_point];
        POINT_COLOR = TRIGGER;
        LCD_DrawLine(x - 8, y, x + 8, y);
        LCD_DrawLine(x, y - 8, x, y + 8);
        draw_calibration = false;
    }
    if (now - touch_at < TOUCH_MS) return;
    touch_at = now;
    bool down = TP_Scan(1) && (tp_dev.sta & TP_PRES_DOWN);
    if (down) {
        cal_samples[cal_point][0] = tp_dev.x[0];
        cal_samples[cal_point][1] = tp_dev.y[0];
    } else if (was_pressed) {
        if (++cal_point == 4U) {
            calibrating = !TP_CalibrateFromPoints(cal_samples, CAL_LEFT, CAL_TOP, CAL_RIGHT, CAL_BOTTOM);
            have_touch = !calibrating;
            cal_point = 0;
            repaint = true;
        }
        draw_calibration = calibrating;
    }
    was_pressed = down;
}

/* 处理按钮点击、阈值拖动和游标拖动。 */
static void scan_touch(uint32_t now)
{
    if (!have_touch || now - touch_at < TOUCH_MS) return;
    touch_at = now;
    bool down = TP_Scan(0) && (tp_dev.sta & TP_PRES_DOWN);
    const ScopeSettings *s = Scope_Settings();
    unsigned x = tp_dev.x[0], y = tp_dev.y[0];
    if (down && !was_pressed && y >= BUTTON_Y && y < BUTTON_Y + 3U * BUTTON_H) {
        switch ((y - BUTTON_Y) / BUTTON_H * 4U + x / BUTTON_W) {
        case 0: if (s->state == SCOPE_FROZEN) Scope_Run(now); else Scope_Stop(); break;
        case 1: Scope_SetMode((TriggerMode)((s->mode + 1U) % 3U), now); break;
        case 2: Scope_CycleTime(); break;
        case 3: Scope_CycleVertical(); break;
        case 4: Scope_SetEdge(s->edge == SCOPE_RISING ? SCOPE_FALLING : SCOPE_RISING); break;
        case 5: set_level = !set_level; break;
        case 6: Scope_SetMode(SCOPE_SINGLE, now); break;
        case 7: Scope_SetDual(!s->dual); break;
        case 8: Scope_SetAutoRange(!s->auto_range); break;
        case 9: Ui_SetSpectrum(!want_spectrum); break;
        case 10: if (s->state != SCOPE_FROZEN) Scope_Stop(); set_level = false; break;
        case 11: Ui_RequestCalibration(); break;
        }
        repaint = true;
        if (footer_row >= 13) footer_row = 13;
    }
    if (down && !want_spectrum && x >= PLOT_X && x < PLOT_X + PLOT_W &&
        y >= PLOT_Y && y < PLOT_Y + PLOT_H) {
        if (set_level) {
            float mv = plot.center_mv + (0.5f - (float)(y - PLOT_Y) / (PLOT_H - 1)) *
                       SCOPE_VERTICAL_DIVS * s->vertical_mv_div;
            Scope_SetTrigger((unsigned)fmaxf(0, fminf(mv, s->reference_mv)));
        } else if (s->state == SCOPE_FROZEN) {
            const int local = (int)(x - PLOT_X);
            if (!was_pressed) {
                drag_cursor = abs(local - (int)cursor_pos[0]) <= abs(local - (int)cursor_pos[1]) ? 0 : 1;
            }
            if (drag_cursor >= 0) cursor_pos[drag_cursor] = (unsigned)local;
        }
    }
    if (!down) drag_cursor = -1;
    was_pressed = down;
}

/* 启动例程 LCD 与触摸校准，检查屏幕尺寸。 */
void Ui_Init(void)
{
    Screen_DelayInit();
    LCD_Init();
    can_draw = lcddev.width == 240 && lcddev.height == 320 && lcddev.id != 0;
    if (!can_draw) return;
    for (unsigned x = 0; x < PLOT_W; ++x) grid_row[x] = x % 21 == 0 ? GRID : BG;
    grid_row[0] = grid_row[PLOT_W - 1] = BORDER;
    LCD_Clear(BG);
    have_touch = !TP_InitWithoutCalibration();
    if (!have_touch) Ui_RequestCalibration();
    plot_row = PLOT_H;
    footer_row = 13;
    repaint = true;
}

/* 完成一次触摸扫描或一次有界的 LCD 绘制任务。 */
void Ui_Poll(uint32_t now)
{
    if (!can_draw) return;
    TP_ServiceCalibration(now);
    if (calibrating) {
        calibrate(now);
        return;
    }
    scan_touch(now);
    if (calibrating) return;
    if (now - fps_start >= 1000U) {
        fps = frame_counter * 1000U / (now - fps_start);
        fps_start = now;
        frame_counter = 0;
    }
    read_panel_scan(now);
    if (plot_row < PLOT_H) {
        unsigned left = PLOT_H - plot_row;
        unsigned count = left < STRIPE_ROWS ? left : STRIPE_ROWS;
        if (!stripe_is_safe(plot_row, count, now)) return;
        paint_stripe(plot_row, count);
        plot_row += count;
        if (plot_row == PLOT_H) ++frame_counter;
    } else if (footer_row < 13) {
        paint_readout(footer_row++);
    } else if (repaint && footer_row < 13 + sizeof buttons / sizeof buttons[0]) {
        paint_button(footer_row++ - 13U);
    } else if ((int32_t)(now - next_display) >= 0) {
        prepare_frame(Scope_Frame(), Scope_Settings(), now);
        next_display = now + SCOPE_FRAME_INTERVAL_MS;
    }
    if (repaint && footer_row == 13 + sizeof buttons / sizeof buttons[0]) repaint = false;
}

/* 切换频谱或时域页面。 */
void Ui_SetSpectrum(bool visible)
{
    want_spectrum = visible;
    repaint = true;
    if (footer_row >= 13) footer_row = 13;
}
/* 返回最近一秒内完整绘制的帧率。 */
unsigned Ui_Fps(void) { return fps; }
/* 返回驱动读到的显示控制器型号。 */
unsigned Ui_LcdId(void) { return lcddev.id; }
/* 返回 ILI9341 扫描行是否稳定回绕。 */
bool Ui_ScanSynchronized(void) { return panel_scan.synchronized; }
/* 返回未能在下一轮扫描前完成写入的帧数。 */
uint32_t Ui_ScanMisses(void) { return panel_scan.misses; }
/* 返回是否加载或完成了触摸校准。 */
bool Ui_TouchReady(void) { return have_touch; }
/* 返回当前是否显示 FFT 页面。 */
bool Ui_IsSpectrum(void) { return want_spectrum; }
