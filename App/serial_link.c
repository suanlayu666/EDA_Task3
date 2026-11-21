#include "serial_link.h"
#include "scope_config.h"
#include "stream_encode.h"
#include "usart.h"
#include "main.h"
#include <string.h>

enum {
    RX_SIZE = 4096, LINE_SIZE = 96, TX_SLOTS = 20, MAX_BODY = 730,
    HEADER_SIZE = 10, TX_RESERVE = 3, STREAM_POINTS = 256, WAVE_POINTS = 240,
    MESSAGE_STREAM = 1, MESSAGE_TEXT = 2, MESSAGE_WAVE = 3, MESSAGE_PARITY = 4, MESSAGE_WEIGHTED_PARITY = 5,
    FIR_HISTORY = 16, FEC_PACKETS = 126, FEC_LANES = 6,
    TX_STREAM_BURST = 2 * FEC_LANES + (SCOPE_DMA_HALF + 2 * STREAM_POINTS - 1) / (2 * STREAM_POINTS),
    TX_COMMAND_LIMIT = TX_SLOTS - TX_RESERVE - TX_STREAM_BURST - 1,
};

_Static_assert(TX_COMMAND_LIMIT > 0, "TX queue must fit one ADC block and command replies");

typedef struct {
    unsigned size;
    uint8_t bytes[HEADER_SIZE + MAX_BODY + 2];
} Packet;

static uint8_t rx[RX_SIZE];
static volatile uint32_t rx_wraps;
static volatile bool rx_restart;
static uint32_t rx_consumed, rx_errors;
static Packet tx[TX_SLOTS];
static volatile uint32_t tx_written, tx_sent;
static volatile bool sending;
static char line[LINE_SIZE];
static unsigned line_size;
static bool discarding;
static uint32_t last_byte_ms, dropped;
static unsigned stream_channels, stream_codes, tap_index, samples_seen;
static uint16_t taps[2][FIR_HISTORY], pending[STREAM_POINTS * 2];
static uint32_t stream_first, group_first;
static unsigned group_packet;
static uint8_t stream_payload[(STREAM_POINTS + STREAM_RANGE_BYTES) * 2];
static uint8_t parity[FEC_LANES][sizeof stream_payload];
static uint8_t weighted_parity[FEC_LANES][sizeof stream_payload];
static uint16_t crc_table[256];

/* 按小端顺序逐字节写入协议字段。 */
static void put16(uint8_t *out, uint16_t value)
{
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8U);
}

/* 将 32 位字段拆成两个小端半字。 */
static void put32(uint8_t *out, uint32_t value)
{
    put16(out, (uint16_t)value);
    put16(out + 2, (uint16_t)(value >> 16U));
}

/* 把两个 12 位样本装入三个字节。 */
static void pack_samples(uint8_t *out, const uint16_t *data, unsigned count)
{
    for (unsigned i = 0; i < count; i += 2U, out += 3) {
        uint16_t first = data[i], second = data[i + 1U];
        out[0] = (uint8_t)first;
        out[1] = (uint8_t)((first >> 8U) | (second << 4U));
        out[2] = (uint8_t)(second >> 4U);
    }
}

/* 读取已发布且尚未发送的包数。 */
static unsigned tx_pending(void)
{
    return tx_written - tx_sent;
}

/* 申请发送槽，并为命令应答留出空间。 */
static Packet *packet_begin(unsigned type, unsigned flags, unsigned length, uint32_t sequence)
{
    if (length > MAX_BODY || tx_pending() >= TX_SLOTS ||
        ((type == MESSAGE_STREAM || type == MESSAGE_PARITY || type == MESSAGE_WEIGHTED_PARITY) && tx_pending() >= TX_SLOTS - TX_RESERVE)) return NULL;

    Packet *packet = &tx[tx_written % TX_SLOTS];
    uint8_t *p = packet->bytes;
    p[0] = 0xa5;
    p[1] = 0x5a;
    p[2] = (uint8_t)type;
    p[3] = (uint8_t)flags;
    put16(p + 4, (uint16_t)length);
    put32(p + 6, sequence);
    packet->size = HEADER_SIZE + length + 2U;
    return packet;
}

/* 填入 CRC 并把完整包加入 UART 发送队列。 */
static void packet_commit(Packet *packet)
{
    uint8_t *p = packet->bytes;
    uint16_t crc = 0xffffU;
    for (unsigned i = 2; i < packet->size - 2U; ++i) {
        crc = (uint16_t)((crc << 8U) ^ crc_table[(crc >> 8U) ^ p[i]]);
    }
    put16(p + packet->size - 2U, crc);
    __DMB();
    ++tx_written;
}

/* 清零 DMA 读取位置并启动循环接收。 */
static void start_rx(void)
{
    rx_wraps = rx_consumed = 0;
    if (HAL_UART_Receive_DMA(&huart1, rx, RX_SIZE) != HAL_OK) Error_Handler();
}

/* 建立 CRC 查表并使能持续接收 DMA。 */
void Serial_Init(void)
{
    for (unsigned i = 0; i < 256; ++i) {
        uint16_t crc = (uint16_t)(i << 8U);
        for (unsigned bit = 0; bit < 8; ++bit) {
            crc = (uint16_t)((crc << 1U) ^ ((crc & 0x8000U) ? 0x1021U : 0U));
        }
        crc_table[i] = crc;
    }
    start_rx();
}

/* 在中断中记录接收 DMA 已写满的次数。 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart1) ++rx_wraps;
}

/* 启动下一包 DMA，调用方负责互斥。 */
static void start_tx(void)
{
    if (sending || !tx_pending()) return;
    Packet *packet = &tx[tx_sent % TX_SLOTS];
    sending = HAL_UART_Transmit_DMA(&huart1, packet->bytes, (uint16_t)packet->size) == HAL_OK;
}

/* 上一包发完后，立即在中断中接着发下一包。 */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart != &huart1) return;
    ++tx_sent;
    sending = false;
    start_tx();
}

/* 记录接收故障，交给前台重启 DMA。 */
void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == &huart1 && uart->RxState == HAL_UART_STATE_READY) rx_restart = true;
}

/* 发送通道空闲且有待发包时启动 DMA。 */
static void service_tx(void)
{
    uint32_t mask = __get_PRIMASK();
    __disable_irq();
    start_tx();
    __set_PRIMASK(mask);
}

/* 接收 ASCII 命令，超长时丢弃整行。 */
static void consume_byte(uint8_t ch, uint32_t now, SerialCommandHandler handler)
{
    last_byte_ms = now;
    if (ch == '\r' || ch == '\n') {
        if (!discarding && line_size) {
            line[line_size] = '\0';
            handler(line, now);
        }
        line_size = 0;
        discarding = false;
    } else if (!discarding) {
        if (line_size == LINE_SIZE - 1U || ((ch < 32 || ch > 126) && ch != '\t')) {
            discarding = true;
            line_size = 0;
            Serial_SendText("ERR");
        } else {
            line[line_size++] = (char)ch;
        }
    }
}

/* 处理有界数量的 DMA 接收字节并服务发送队列。 */
void Serial_Poll(uint32_t now, SerialCommandHandler handler)
{
    service_tx();
    if (rx_restart) {
        rx_restart = false;
        ++rx_errors;
        line_size = 0;
        discarding = true;
        last_byte_ms = now;
        start_rx();
        Serial_SendText("ERR");
    }

    uint32_t wraps, offset;
    do {
        wraps = rx_wraps;
        offset = RX_SIZE - __HAL_DMA_GET_COUNTER(huart1.hdmarx);
        __DMB();
    } while (wraps != rx_wraps);
    uint32_t produced = wraps * RX_SIZE + offset;
    /* NDTR can reload immediately before the wrap callback runs. */
    if ((int32_t)(produced - rx_consumed) >= 0 && produced - rx_consumed > RX_SIZE) {
        ++rx_errors;
        rx_consumed = produced;
        line_size = 0;
        discarding = true;
        last_byte_ms = now;
        Serial_SendText("ERR");
    }
    /* 为下一批采样包和恢复包留槽位。WAVE? 一条命令最多占两个槽。 */
    for (unsigned i = 0; i < 128U && (int32_t)(produced - rx_consumed) > 0 &&
         tx_pending() < TX_COMMAND_LIMIT; ++i) {
        consume_byte(rx[rx_consumed++ & (RX_SIZE - 1U)], now, handler);
    }
    if (produced == rx_consumed && now - last_byte_ms >= 250U &&
        (!line_size || tx_pending() < TX_COMMAND_LIMIT)) {
        if (line_size) Serial_SendText("ERR");
        line_size = 0;
        discarding = false;
    }
    service_tx();
}

/* 配置实时串流路数并清除旧滤波状态。 */
bool Serial_SetStream(unsigned channels)
{
    if (channels > 2) return false;
    if (channels != stream_channels) {
        stream_channels = channels;
        stream_codes = tap_index = samples_seen = group_packet = 0;
    }
    return true;
}

/* 用半带 FIR 滤波，将单路抽取到 100 kS/s。 */
static uint16_t halfband(const uint16_t *data, unsigned at)
{
    const int weights[] = {-120, 530, -2242, 9993};
    int sum = data[(at - 7U) & 15U] * 16446;
    for (unsigned i = 0; i < 4; ++i) {
        unsigned distance = i * 2U;
        sum += weights[i] * (data[(at - distance) & 15U] + data[(at - 14U + distance) & 15U]);
    }
    sum = (sum + 16384) >> 15;
    return (uint16_t)(sum < 0 ? 0 : sum > 4095 ? 4095 : sum);
}

/* 用对称低通将双路抽取到每路 50 kS/s。 */
static uint16_t quarterband(const uint16_t *data, unsigned at)
{
    static const int coeff[] = {-42, -177, -406, -352, 669, 2961, 5846, 7885};
    int sum = 0;
    for (unsigned i = 0; i < 8; ++i) {
        sum += coeff[i] * (data[(at - i) & 15U] + data[(at - 15U + i) & 15U]);
    }
    sum = (sum + 16384) / 32768;
    return (uint16_t)(sum < 0 ? 0 : sum > 4095 ? 4095 : sum);
}

/* 发布一帧恢复包。CRC 与普通串流包一致。 */
static void send_parity(unsigned type, unsigned lane, const uint8_t *payload, unsigned bytes)
{
    unsigned flags = (stream_channels - 1U) | (lane << 1U);
    Packet *recovery = packet_begin(type, flags, bytes, group_first);
    if (recovery) {
        memcpy(recovery->bytes + HEADER_SIZE, payload, bytes);
        packet_commit(recovery);
    } else {
        ++dropped;
    }
}

/* 把串流包交错分入六组，每组生成两个恢复包。 */
static void stream_packet(void)
{
    unsigned bytes = Stream_Pack(stream_payload, pending, STREAM_POINTS, stream_channels);
    if (!group_packet) {
        group_first = stream_first;
        memset(parity, 0, sizeof parity);
        memset(weighted_parity, 0, sizeof weighted_parity);
    }
    unsigned lane = group_packet % FEC_LANES;
    for (unsigned i = 0; i < bytes; ++i) {
        uint8_t q = weighted_parity[lane][i];
        uint8_t q_times_two = (uint8_t)((q << 1U) ^ ((q & 0x80U) ? 0x1dU : 0));
        parity[lane][i] ^= stream_payload[i];
        weighted_parity[lane][i] = q_times_two ^ stream_payload[i];
    }

    unsigned flags = (stream_channels - 1U) | (group_packet << 1U);
    Packet *data = packet_begin(MESSAGE_STREAM, flags, bytes, stream_first);
    if (data) {
        memcpy(data->bytes + HEADER_SIZE, stream_payload, bytes);
        packet_commit(data);
    } else {
        ++dropped;
    }

    if (++group_packet == FEC_PACKETS) {
        for (unsigned check_lane = 0; check_lane < FEC_LANES; ++check_lane) {
            send_parity(MESSAGE_PARITY, check_lane, parity[check_lane], bytes);
            send_parity(MESSAGE_WEIGHTED_PARITY, check_lane, weighted_parity[check_lane], bytes);
        }
        group_packet = 0;
    }
}

/* 跨 DMA 块抽取并打包，保持均匀的样点间隔。 */
void Serial_StreamBlock(const SampleBlock *block)
{
    if (!stream_channels) return;
    if (block->discontinuity) stream_codes = tap_index = samples_seen = group_packet = 0;
    unsigned stride = stream_channels == 1 ? 2U : 4U;
    for (unsigned i = 0; i < block->count; ++i) {
        unsigned at = tap_index++ & (FIR_HISTORY - 1U);
        for (unsigned ch = 0; ch < stream_channels; ++ch) {
            taps[ch][at] = Acquisition_Code(block->data[i], ch);
        }
        if (samples_seen < FIR_HISTORY) ++samples_seen;
        uint32_t index = block->first + i;
        if (samples_seen < FIR_HISTORY || index % stride != 0) continue;
        if (stream_codes == 0) stream_first = index / stride;
        for (unsigned ch = 0; ch < stream_channels; ++ch) {
            pending[stream_codes++] = stride == 2U ? halfband(taps[ch], at) : quarterband(taps[ch], at);
        }
        if (stream_codes < STREAM_POINTS * stream_channels) continue;

        stream_packet();
        stream_codes = 0;
    }
}

/* 把应答或读数作为独立消息入队。 */
bool Serial_SendText(const char *text)
{
    unsigned length = strlen(text);
    Packet *p = packet_begin(MESSAGE_TEXT, 0, length, 0);
    if (!p) return false;
    memcpy(p->bytes + HEADER_SIZE, text, length);
    packet_commit(p);
    return true;
}

/* 等间隔取样，发送覆盖整屏的捕获快照。 */
bool Serial_SendFrame(const ScopeFrame *frame, unsigned channels)
{
    if (frame->count < 2U || channels < 1U || channels > 2U) return false;
    Packet *p = packet_begin(MESSAGE_WAVE, channels, 10U + WAVE_POINTS * channels * 3U / 2U,
                             frame->sequence);
    if (!p) return false;
    uint8_t *out = p->bytes + HEADER_SIZE;
    put32(out, frame->time_us_div * SCOPE_HORIZONTAL_DIVS);
    put16(out + 4, WAVE_POINTS);
    put16(out + 6, (uint16_t)frame->reference_mv);
    put16(out + 8, frame->triggered ? 1U : 0U);
    uint16_t codes[WAVE_POINTS * 2];
    for (unsigned i = 0; i < WAVE_POINTS; ++i) {
        float index = frame->start_fraction + (frame->count - 2U) * (float)i / (WAVE_POINTS - 1U);
        for (unsigned ch = 0; ch < channels; ++ch) {
            codes[i * channels + ch] = (uint16_t)(Scope_Interpolate(frame, index, ch) + 0.5f);
        }
    }
    pack_samples(out + 10, codes, WAVE_POINTS * channels);
    packet_commit(p);
    return true;
}

/* 返回实时串流的路数。 */
unsigned Serial_StreamChannels(void) { return stream_channels; }
/* 返回实时串流发送因积压丢弃的包数。 */
uint32_t Serial_Dropped(void) { return dropped; }
/* 返回接收溢出和硬件错误总数。 */
uint32_t Serial_RxErrors(void) { return rx_errors; }
