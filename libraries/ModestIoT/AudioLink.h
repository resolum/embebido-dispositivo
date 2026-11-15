#ifndef AUDIO_LINK_H
#define AUDIO_LINK_H

/**
 * @file AudioLink.h
 * @brief Transport-agnostic contracts shared by the audio link (StreamGatewayClient) and the speaker.
 * @ingroup network_module
 *
 * Header-only and free of Arduino dependencies so it can be unit-tested on a PC.
 */

#include <stddef.h>
#include <stdint.h>

/** @brief Receives the speaker side of the protocol (edge -> ESP32). Called from the link task: be quick. */
class AudioLinkListener {
public:
    virtual ~AudioLinkListener() = default;
    virtual void onAudioBegin(const char* utteranceId, uint32_t sampleRate, bool emergency) = 0;
    virtual void onAudioData(const uint8_t* pcm, size_t length) = 0;
    virtual void onAudioEnd(const char* utteranceId) = 0;
    virtual void onAudioCancel(const char* utteranceId) = 0;
    /** @brief The link dropped (or never came up): stop sound and empty buffers. */
    virtual void onLinkDown() = 0;
};

/** @brief Lets the speaker tell the edge that an utterance has finished sounding. */
class PlaybackReporter {
public:
    virtual ~PlaybackReporter() = default;
    virtual bool sendPlaybackDone(const char* utteranceId) = 0;
};

enum class LinkState : uint8_t {
    CARRIER_DOWN,  ///< No WiFi / no Bluetooth peer yet.
    DISCONNECTED,  ///< Carrier up, protocol session not open (waiting for the next attempt).
    CONNECTING,    ///< Session opened, waiting for the edge's "ready".
    READY          ///< hello accepted: audio can flow.
};

struct LinkStats {
    uint32_t bytesSent = 0;
    uint32_t bytesReceived = 0;
    uint32_t micFramesSent = 0;
    uint32_t micFramesDroppedNoLink = 0;  ///< Mic frames thrown away because the link was not ready.
    uint32_t micFramesDroppedSendFail = 0;
    uint32_t micQueueDrops = 0;           ///< Oldest frames evicted by the mic queue (consumer late).
    uint32_t reconnects = 0;
    uint32_t handshakeFailures = 0;
    uint32_t rxErrors = 0;                ///< Bluetooth: bytes/frames discarded (resync, CRC, ring overflow).
};

#endif // AUDIO_LINK_H
