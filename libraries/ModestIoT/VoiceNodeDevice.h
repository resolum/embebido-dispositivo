#ifndef VOICE_NODE_DEVICE_H
#define VOICE_NODE_DEVICE_H

/**
 * @file VoiceNodeDevice.h
 * @brief A complete voice node as one ModestIoT Device: INMP441 microphone + MAX98357A speaker + edge link.
 * @ingroup core_module
 *
 * The sketch only builds the parts and calls begin() / service():
 *
 *   hardware timer -> HardwareWorker -> microphone.on(MEASURE_DATA_REQUESTED_EVENT)
 *     -> SOUND_STARTED / SOUND_ENDED -> VoiceActivityTelemetry
 *     -> DATA_READ                   -> AudioLevelTelemetry
 *   telemetry queue -> NetworkWorker -> processQueuedTelemetryData() -> Serial and, when ready, the edge
 *   microphone frame queue -> uplink task (core 0) -> link.sendAudio()     (PCM to the edge)
 *   link (StreamGatewayClient: WiFi or Bluetooth) -> speaker (AudioLinkListener) -> I2S -> MAX98357A
 *
 * The link is any StreamGatewayClient, so switching between WiFi/WebSocket and Bluetooth is a one-line
 * change in the sketch. The audio itself never goes through the telemetry queue: the capture task in
 * Inmp441Microphone and the uplink task keep the 16 kHz stream from ever being late.
 *
 * Serial console (one letter + Enter): h help, s status, v live meter, j JSON, + / - gain, w link status,
 * t speaker test tone, r 3 s analysed recording, R WAV dump for tools/grabar_wav.py.
 */

#include "Device.h"
#include "Inmp441Microphone.h"
#include "Max98357Speaker.h"
#include "StreamGatewayClient.h"

/** @brief Tunables of the node. Defaults are the values used by Alivia Voice. */
struct VoiceNodeSettings {
    float microphoneGainDb = 24.0f;
    float voiceThresholdDb = 12.0f;                      ///< dB above the noise floor that counts as voice
    unsigned long voiceHangoverMs = 600;                 ///< silence needed to close a voice segment
    unsigned long levelReportIntervalMs = 1000;          ///< one level line per second
    Inmp441Microphone::Channel microphoneChannel = Inmp441Microphone::Channel::AUTO;
    size_t micQueueFrames = 25;                          ///< 25 x 640 B = 16 KB, ~500 ms
    size_t framesPerMessage = 2;                         ///< 1..4 frames of 20 ms per message (2 = 40 ms)
    UBaseType_t uplinkTaskPriority = 2;
    uint32_t uplinkTaskStackBytes = 3072;
    BaseType_t uplinkTaskCore = 0;
    bool sendAudioLevelTelemetry = true;                 ///< voice_activity is always sent
    uint32_t testRecordingSeconds = 3;                   ///< 'r'
    uint32_t wavRecordingSeconds = 3;                    ///< 'R'
    uint8_t telemetryQueueLength = 16;
};

class VoiceNodeDevice : public Device {
public:
    VoiceNodeDevice(const char* deviceIdentifier, Inmp441Microphone& microphoneSensor, Max98357Speaker& speakerActuator,
                    StreamGatewayClient& edgeLink, unsigned long schedulerTickInMilliseconds,
                    const VoiceNodeSettings& nodeSettings = VoiceNodeSettings());

    /**
     * @brief Starts microphone, framework tasks, speaker, link and uplink. Call from setup() after
     *        SerialConsole::begin(). Returns false only if the microphone could not start (the node then idles).
     */
    bool begin();

    /** @brief Call from loop(): serial commands and the live meter. Does not block. */
    void service();

    void on(Event event) override;
    void handle(Command command) override;

    // Console actions (also usable from code)
    void handleSerialCommand(char command);
    void printHelp();
    void printStatus();
    void printLinkStatus();
    void changeGain(float deltaInDecibels);
    void runTestRecording();
    void sendWavRecording();
    void playTestTone();
    void stopPlayback();

    void setJsonOutputEnabled(bool enabled) { jsonOutputEnabled = enabled; }
    [[nodiscard]] bool isJsonOutputEnabled() const { return jsonOutputEnabled; }
    /** @brief Hides the periodic level lines (used while the live VU meter is on). */
    void setReportsVisible(bool visible) { reportsVisible = visible; }
    [[nodiscard]] bool isReady() const { return microphoneReady; }

protected:
    void processQueuedTelemetryData(const TelemetryPackage* telemetryPackage) const override;

private:
    static void uplinkTaskEntry(void* parameters);
    void uplinkLoop();
    int16_t* allocateRecordingBuffer(uint32_t seconds, size_t& sampleCount);
    size_t recordInto(int16_t* buffer, size_t sampleCount);
    void printLiveMeter();

    const char* deviceId;
    Inmp441Microphone& microphone;
    Max98357Speaker& speaker;
    StreamGatewayClient& link;
    const VoiceNodeSettings settings;

    TaskHandle_t uplinkTaskHandle = nullptr;
    volatile bool jsonOutputEnabled = false;
    volatile bool reportsVisible = true;
    bool liveMeterEnabled = false;
    bool microphoneReady = false;
    unsigned long lastMeterTimestamp = 0;
    unsigned long tickInMilliseconds;
};

#endif // VOICE_NODE_DEVICE_H
