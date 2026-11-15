#ifndef MAX98357_SPEAKER_H
#define MAX98357_SPEAKER_H

/**
 * @file Max98357Speaker.h
 * @brief MAX98357A I2S amplifier + speaker as a ModestIoT Actuator, fed by the edge over WiFi or Bluetooth.
 * @ingroup actuators_module
 *
 * Second I2SClass instance in TX mode (16 bit, mono, same sample on both slots); the INMP441 uses the other
 * I2S port of the classic ESP32. Audio arrives as audio_begin / PCM messages / audio_end through the
 * AudioLinkListener callbacks (any StreamGatewayClient) and is stored in a small jitter buffer; a playback
 * task on core 0 starts after a prefill, fills gaps with silence on underrun and reports playback_done
 * (through the PlaybackReporter, normally the link) when the last sample has sounded.
 *
 * Commands (Actuator): PLAY_TEST_TONE_COMMAND plays a 440 Hz tone to check the wiring, STOP_COMMAND cuts sound.
 */

#include <ESP_I2S.h>

#include "Actuator.h"
#include "AudioLink.h"

/** @brief Tunables of the speaker. Defaults are the values used by Alivia Voice. */
struct SpeakerSettings {
    uint8_t volumePercent = 60;            ///< Ceiling for the output level (0..100).
    uint32_t bufferMs = 500;               ///< Jitter buffer capacity (16 KB at 16 kHz).
    uint32_t prefillMs = 200;              ///< Audio buffered before sound starts.
    UBaseType_t taskPriority = 4;
    uint32_t taskStackBytes = 4096;
    BaseType_t taskCore = 0;
    uint32_t testToneHz = 440;             ///< 't' command.
    uint32_t testToneMs = 1000;
};

struct SpeakerStats {
    uint32_t utterances = 0;
    uint32_t underruns = 0;
    uint32_t overflowBytes = 0;   ///< Bytes dropped because the jitter buffer was full.
    uint32_t cancelled = 0;
    uint32_t bufferedBytes = 0;
    uint32_t bufferCapacityBytes = 0;
    bool playing = false;
};

class Max98357Speaker : public Actuator, public AudioLinkListener {
public:
    static constexpr int PLAY_TEST_TONE_COMMAND_IDENTIFIER = 300;
    static constexpr int STOP_COMMAND_IDENTIFIER = 301;
    static constexpr Command PLAY_TEST_TONE_COMMAND{PLAY_TEST_TONE_COMMAND_IDENTIFIER};
    static constexpr Command STOP_COMMAND{STOP_COMMAND_IDENTIFIER};

    Max98357Speaker(int bclkPin, int lrcPin, int dinPin, uint32_t sampleRateInHertz,
                    const SpeakerSettings& speakerSettings = SpeakerSettings(),
                    CommandHandler* parentHandler = nullptr);
    ~Max98357Speaker() override;

    /** @brief Starts I2S TX, allocates the jitter buffer and starts the playback task (core 0). */
    bool begin();

    /** @brief Who receives playback_done (normally the StreamGatewayClient). Set before audio can arrive. */
    void setPlaybackReporter(PlaybackReporter* reporter) { playbackReporter = reporter; }

    void setVolumePercent(uint8_t percent);
    [[nodiscard]] int getBclkPin() const { return bclk; }
    [[nodiscard]] int getLrcPin() const { return lrc; }
    [[nodiscard]] int getDinPin() const { return din; }
    [[nodiscard]] uint32_t getSampleRate() const { return sampleRate; }
    [[nodiscard]] uint8_t getVolumePercent() const;
    [[nodiscard]] SpeakerStats stats() const;

    // AudioLinkListener (called from the link task)
    void onAudioBegin(const char* utteranceId, uint32_t sampleRate, bool emergency) override;
    void onAudioData(const uint8_t* pcm, size_t length) override;
    void onAudioEnd(const char* utteranceId) override;
    void onAudioCancel(const char* utteranceId) override;
    void onLinkDown() override;

protected:
    void executeCommand(Command command) override;

private:
    static constexpr size_t ID_CAPACITY = 40;
    static constexpr size_t CHUNK_SAMPLES = 320;  // 20 ms at 16 kHz

    static void taskEntry(void* parameters);
    void run();
    void playUtterance();
    void playTone();
    void writeSamples(int16_t* samples, size_t count);
    void writeSilence(size_t samples);
    void stopNow();
    size_t ringSize() const;
    size_t ringPush(const uint8_t* data, size_t length);
    size_t ringPop(uint8_t* destination, size_t maximum);

    I2SClass i2s;
    const int bclk;
    const int lrc;
    const int din;
    const uint32_t sampleRate;
    const SpeakerSettings settings;
    PlaybackReporter* playbackReporter = nullptr;
    TaskHandle_t taskHandle = nullptr;
    volatile uint8_t volumePercent;

    // Jitter buffer: a byte ring shared by the link task (producer) and the playback task (consumer).
    uint8_t* ring = nullptr;
    size_t ringCapacity = 0;
    size_t ringHead = 0;
    size_t ringTail = 0;
    size_t ringCount = 0;
    mutable portMUX_TYPE ringLock = portMUX_INITIALIZER_UNLOCKED;

    // Utterance state, guarded by the same spinlock.
    char utteranceId[ID_CAPACITY] = {};
    uint32_t generation = 0;
    bool active = false;
    bool ended = false;
    volatile bool toneRequested = false;
    volatile bool stopRequested = false;
    SpeakerStats counters;
    int16_t chunk[CHUNK_SAMPLES] = {};
};

#endif // MAX98357_SPEAKER_H
