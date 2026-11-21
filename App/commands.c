#include "commands.h"
#include "serial_link.h"
#include "ui.h"
#include "main.h"
#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* 跳过命令前缀或参数之间的空白。 */
static const char *skip_space(const char *s)
{
    while (isspace((unsigned char)*s)) ++s;
    return s;
}

/* 匹配完整参数，忽略大小写和尾部空白。 */
static bool is_word(const char *s, const char *expected)
{
    s = skip_space(s);
    while (*expected && toupper((unsigned char)*s) == toupper((unsigned char)*expected)) {
        ++s;
        ++expected;
    }
    return *expected == '\0' && *skip_space(s) == '\0';
}

/* 解析数值及单位，换算成毫伏或微秒。 */
static bool parse_quantity(const char *s, bool is_time, unsigned *value)
{
    char *end;
    float number = strtof(s, &end);
    if (end == s || !isfinite(number) || number < 0) return false;
    if (is_time) {
        if (is_word(end, "ms")) number *= 1000;
        else if (!is_word(end, "") && !is_word(end, "us")) return false;
    } else {
        if (is_word(end, "") || is_word(end, "V")) number *= 1000;
        else if (!is_word(end, "mV")) return false;
    }
    if (number > 100000 || fabsf(number - roundf(number)) > 0.01f) return false;
    *value = (unsigned)lroundf(number);
    return true;
}

/* 返回屏幕捕获帧中保存的测量值。 */
static void print_measurement(const ScopeFrame *frame)
{
    const Measurement *a = &frame->measurement.channel[0], *b = &frame->measurement.channel[1];
    char message[250];
    if (!frame->measurement.sequence) {
        Serial_SendText("OK NO MEASUREMENT");
        return;
    }
    int length = snprintf(message, sizeof message, "OK FRAME=%lu CH1 %.1f Hz %.3f Vpp %.3f V DC %s",
                          (unsigned long)frame->sequence, a->frequency_hz, a->peak_to_peak_mv / 1000.0f,
                          a->dc_mv / 1000.0f, Measurement_Name(a->type));
    if (Scope_Settings()->dual && length > 0 && (unsigned)length < sizeof message) {
        char *p = message + length;
        size_t remain = sizeof message - (size_t)length;
        int appended = snprintf(p, remain, " | CH2 %.1f Hz %.3f Vpp %.3f V DC %s",
                                b->frequency_hz, b->peak_to_peak_mv / 1000.0f,
                                b->dc_mv / 1000.0f, Measurement_Name(b->type));
        if (frame->measurement.phase_valid && appended > 0 && (unsigned)appended < remain) {
            snprintf(p + appended, remain - (size_t)appended, " | PHASE %.1f deg", frame->measurement.phase_deg);
        }
    }
    Serial_SendText(message);
}

/* 读取当前设置、测量值和硬件状态。 */
static bool query(const char *command, uint32_t now)
{
    const ScopeSettings *s = Scope_Settings();
    const ScopeFrame *frame = Scope_Frame();
    char message[250];
    if (!strcmp(command, "MEAS?")) {
        print_measurement(frame);
        return true;
    }
    if (!strcmp(command, "WAVE?")) {
        Serial_SendText(Serial_SendFrame(frame, s->dual ? 2U : 1U) ? "OK" : "ERR");
        return true;
    }
    if (!strcmp(command, "CONF?")) {
        snprintf(message, sizeof message,
                 "OK %s %s EDGE=%s TB=%uus VDIV=%umV TRIG=%umV VREF=%umV CH=%u RANGE=%s PAGE=%s STREAM=%u",
                 Scope_ModeName(), Scope_StateName(), s->edge == SCOPE_RISING ? "RISE" : "FALL",
                 s->time_us_div, s->vertical_mv_div, s->trigger_mv, s->reference_mv, s->dual ? 2U : 1U,
                 s->auto_range ? "AUTO" : "MANUAL", Ui_IsSpectrum() ? "FFT" : "SCOPE", Serial_StreamChannels());
    } else if (!strcmp(command, "THD?")) {
        if (!frame->spectrum.valid) {
            Serial_SendText("OK NO SPECTRUM");
            return true;
        }
        const Spectrum *v = &frame->spectrum;
        snprintf(message, sizeof message, "OK THD=%.3f%% A1=%.3f A2=%.3f A3=%.3f A4=%.3f A5=%.3f mVpk",
                 v->thd_percent, v->harmonic_mv[0], v->harmonic_mv[1], v->harmonic_mv[2],
                 v->harmonic_mv[3], v->harmonic_mv[4]);
    } else if (!strcmp(command, "STAT?")) {
        snprintf(message, sizeof message,
                 "OK MS=%lu HCLK=%lu FS=%u ADC=%lu ADCERR=%lu DMADROP=%lu TXDROP=%lu RXERR=%lu LCD=%04X TOUCH=%u FPS=%u FRAME=%lu LCDSYNC=%u LCDMISS=%lu",
                 (unsigned long)now, (unsigned long)HAL_RCC_GetHCLKFreq(), SCOPE_SAMPLE_HZ,
                 (unsigned long)Acquisition_End(), (unsigned long)Acquisition_Errors(),
                 (unsigned long)Acquisition_Dropped(), (unsigned long)Serial_Dropped(),
                 (unsigned long)Serial_RxErrors(), Ui_LcdId(), Ui_TouchReady() ? 1U : 0U,
                 Ui_Fps(), (unsigned long)frame->sequence,
                 Ui_ScanSynchronized() ? 1U : 0U, (unsigned long)Ui_ScanMisses());
    } else {
        return false;
    }

    Serial_SendText(message);
    return true;
}

/* 完整校验设置指令，再更新参数。 */
static bool apply(const char *command, const char *arg, uint32_t now)
{
    unsigned value;
    if (!strcmp(command, "RUN") && is_word(arg, "")) Scope_Run(now);
    else if (!strcmp(command, "STOP") && is_word(arg, "")) Scope_Stop();
    else if (!strcmp(command, "SINGLE") && is_word(arg, "")) Scope_SetMode(SCOPE_SINGLE, now);
    else if (!strcmp(command, "TRIG")) return parse_quantity(arg, false, &value) && Scope_SetTrigger(value);
    else if (!strcmp(command, "TB")) return parse_quantity(arg, true, &value) && Scope_SetTime(value);
    else if (!strcmp(command, "VDIV")) return parse_quantity(arg, false, &value) && Scope_SetVertical(value);
    else if (!strcmp(command, "VREF")) return parse_quantity(arg, false, &value) && Scope_SetReference(value);
    else if (!strcmp(command, "EDGE")) {
        if (is_word(arg, "RISE")) Scope_SetEdge(SCOPE_RISING);
        else if (is_word(arg, "FALL")) Scope_SetEdge(SCOPE_FALLING);
        else return false;
    } else if (!strcmp(command, "MODE")) {
        if (is_word(arg, "AUTO")) Scope_SetMode(SCOPE_AUTO, now);
        else if (is_word(arg, "NORMAL")) Scope_SetMode(SCOPE_NORMAL, now);
        else if (is_word(arg, "SINGLE")) Scope_SetMode(SCOPE_SINGLE, now);
        else return false;
    } else if (!strcmp(command, "CHAN")) {
        if (is_word(arg, "1")) Scope_SetDual(false);
        else if (is_word(arg, "2")) Scope_SetDual(true);
        else return false;
    } else if (!strcmp(command, "STREAM")) {
        if (is_word(arg, "OFF")) return Serial_SetStream(0);
        if (is_word(arg, "1")) return Serial_SetStream(1);
        if (is_word(arg, "2")) return Serial_SetStream(2);
        return false;
    } else if (!strcmp(command, "RANGE")) {
        if (is_word(arg, "AUTO")) Scope_SetAutoRange(true);
        else if (is_word(arg, "MANUAL")) Scope_SetAutoRange(false);
        else return false;
    } else if (!strcmp(command, "PAGE")) {
        if (is_word(arg, "FFT")) Ui_SetSpectrum(true);
        else if (is_word(arg, "SCOPE")) Ui_SetSpectrum(false);
        else return false;
    } else if (!strcmp(command, "CAL") && is_word(arg, "")) {
        Ui_RequestCalibration();
    } else {
        return false;
    }
    return true;
}

/* 切分指令名和参数，按查询或设置分发。 */
void Commands_Execute(const char *line, uint32_t now)
{
    char command[12];
    unsigned length = 0;
    const char *p = skip_space(line);
    while (*p && !isspace((unsigned char)*p) && length < sizeof command - 1U) {
        command[length++] = (char)toupper((unsigned char)*p++);
    }
    command[length] = '\0';
    if (!(*p && !isspace((unsigned char)*p))) {
        const char *arg = skip_space(p);
        if (!*arg && query(command, now)) return;
        if (apply(command, arg, now)) {
            Serial_SendText("OK");
            return;
        }
    }
    Serial_SendText("ERR");
}
