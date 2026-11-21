#include "stream_encode.h"

/* 写入每路 12 位最小码和跨度，再写该路的 8 位相对码。 */
unsigned Stream_Pack(uint8_t *out, const uint16_t *samples, unsigned points, unsigned channels)
{
    uint8_t *codes = out + STREAM_RANGE_BYTES * channels;
    for (unsigned channel = 0; channel < channels; ++channel) {
        unsigned low = samples[channel], high = low;
        for (unsigned i = channel; i < points * channels; i += channels) {
            unsigned code = samples[i];
            if (code < low) low = code;
            if (code > high) high = code;
        }
        unsigned span = high - low;
        uint8_t *header = out + STREAM_RANGE_BYTES * channel;
        header[0] = (uint8_t)low;
        header[1] = (uint8_t)(low >> 8U);
        header[2] = (uint8_t)span;
        header[3] = (uint8_t)(span >> 8U);
        for (unsigned i = channel; i < points * channels; i += channels) {
            codes[i] = span ? (uint8_t)(((samples[i] - low) * 255U + span / 2U) / span) : 0;
        }
    }
    return (STREAM_RANGE_BYTES + points) * channels;
}
