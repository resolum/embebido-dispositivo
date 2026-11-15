#ifndef STREAM_FRAME_CODEC_H
#define STREAM_FRAME_CODEC_H

/**
 * @file StreamFrameCodec.h
 * @brief Framing used on byte-stream transports (Bluetooth Classic SPP) to carry the Alivia protocol.
 * @ingroup network_module
 *
 * WebSocket already delimits messages; a serial byte stream does not. Frame layout:
 *
 *   0xA7 | type | lenLo | lenHi | hdrXor | payload[len] | crcLo | crcHi
 *
 *   hdrXor = 0xA7 ^ type ^ lenLo ^ lenHi         (cheap guard against false sync bytes)
 *   crc    = CRC-16/CCITT-FALSE of the payload   (poly 0x1021, init 0xFFFF)
 *   type   = 0x01 JSON text, 0x02 PCM audio, 0x03 PING, 0x04 PONG
 *   len    <= 2048
 *
 * The decoder recovers from lost or corrupt bytes by dropping one byte and re-scanning for the next
 * valid header, so a glitch costs one frame (20-40 ms of audio), never the whole session.
 * Header-only, no Arduino dependency: it is unit-tested on a PC against the Python edge implementation.
 */

#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace StreamFrame {

constexpr uint8_t SYNC = 0xA7;
constexpr size_t HEADER_SIZE = 5;
constexpr size_t TRAILER_SIZE = 2;
constexpr size_t MAX_PAYLOAD = 2048;
constexpr size_t MAX_FRAME = HEADER_SIZE + MAX_PAYLOAD + TRAILER_SIZE;

constexpr uint8_t TYPE_TEXT = 0x01;
constexpr uint8_t TYPE_AUDIO = 0x02;
constexpr uint8_t TYPE_PING = 0x03;
constexpr uint8_t TYPE_PONG = 0x04;

inline bool validType(uint8_t type) { return type >= TYPE_TEXT && type <= TYPE_PONG; }

inline uint16_t crc16(const uint8_t* data, size_t length) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            crc = (crc & 0x8000) ? static_cast<uint16_t>((crc << 1) ^ 0x1021) : static_cast<uint16_t>(crc << 1);
        }
    }
    return crc;
}

/** @brief Writes one frame into `out`. Returns the frame size, or 0 if it does not fit / payload too long. */
inline size_t encode(uint8_t type, const uint8_t* payload, size_t length, uint8_t* out, size_t capacity) {
    if (!validType(type) || length > MAX_PAYLOAD || capacity < HEADER_SIZE + length + TRAILER_SIZE) return 0;
    if (length > 0 && payload == nullptr) return 0;
    out[0] = SYNC;
    out[1] = type;
    out[2] = static_cast<uint8_t>(length & 0xFF);
    out[3] = static_cast<uint8_t>(length >> 8);
    out[4] = static_cast<uint8_t>(SYNC ^ out[1] ^ out[2] ^ out[3]);
    if (length > 0) memcpy(out + HEADER_SIZE, payload, length);
    const uint16_t crc = crc16(out + HEADER_SIZE, length);
    out[HEADER_SIZE + length] = static_cast<uint8_t>(crc & 0xFF);
    out[HEADER_SIZE + length + 1] = static_cast<uint8_t>(crc >> 8);
    return HEADER_SIZE + length + TRAILER_SIZE;
}

/** @brief Incremental decoder. Feed it raw bytes in any chunking; complete frames come out through the handler. */
class Decoder {
public:
    using Handler = void (*)(void* context, uint8_t type, const uint8_t* payload, size_t length);

    uint32_t framesOk = 0;
    uint32_t skippedBytes = 0;  ///< Bytes discarded while searching for a header.
    uint32_t badHeaders = 0;
    uint32_t crcErrors = 0;

    void reset() { used = 0; }

    /** @brief The payload pointer given to the handler is only valid during the call. Returns frames delivered. */
    size_t feed(const uint8_t* data, size_t length, Handler handler, void* context) {
        size_t delivered = 0;
        while (length > 0) {
            const size_t room = sizeof(buffer) - used;
            const size_t take = length < room ? length : room;
            memcpy(buffer + used, data, take);
            used += take;
            data += take;
            length -= take;
            delivered += process(handler, context);
        }
        return delivered;
    }

private:
    size_t process(Handler handler, void* context) {
        size_t position = 0;
        size_t delivered = 0;
        while (used - position > 0) {
            if (buffer[position] != SYNC) {
                ++position;
                ++skippedBytes;
                continue;
            }
            if (used - position < HEADER_SIZE) break;
            const uint8_t* header = buffer + position;
            const size_t length = static_cast<size_t>(header[2]) | (static_cast<size_t>(header[3]) << 8);
            const uint8_t check = static_cast<uint8_t>(SYNC ^ header[1] ^ header[2] ^ header[3]);
            if (!validType(header[1]) || length > MAX_PAYLOAD || check != header[4]) {
                ++position;
                ++badHeaders;
                continue;
            }
            const size_t total = HEADER_SIZE + length + TRAILER_SIZE;
            if (used - position < total) break;
            const uint16_t expected = static_cast<uint16_t>(header[HEADER_SIZE + length]) |
                                      static_cast<uint16_t>(header[HEADER_SIZE + length + 1] << 8);
            if (crc16(header + HEADER_SIZE, length) != expected) {
                ++position;
                ++crcErrors;
                continue;
            }
            ++framesOk;
            ++delivered;
            if (handler != nullptr) handler(context, header[1], header + HEADER_SIZE, length);
            position += total;
        }
        if (position > 0) {
            memmove(buffer, buffer + position, used - position);
            used -= position;
        }
        return delivered;
    }

    uint8_t buffer[MAX_FRAME] = {};
    size_t used = 0;
};

}  // namespace StreamFrame

#endif // STREAM_FRAME_CODEC_H
