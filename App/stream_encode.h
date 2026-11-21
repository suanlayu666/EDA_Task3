#ifndef STREAM_ENCODE_H
#define STREAM_ENCODE_H

#include <stdint.h>

enum { STREAM_RANGE_BYTES = 4 };

/* 将一包交错的 ADC 数据按各通道量程编码，返回负载字节数。 */
unsigned Stream_Pack(uint8_t *out, const uint16_t *samples, unsigned points, unsigned channels);

#endif
