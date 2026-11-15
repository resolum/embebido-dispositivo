/**
 * @file VoiceNodeDevice.cpp
 * @brief Voice node device: scheduler wiring, telemetry dispatch and microphone uplink.
 * @ingroup core_module
 */

#include "VoiceNodeDevice.h"

#include <ArduinoJson.h>
#include "AudioTelemetry.h"
#include "SerialConsole.h"

namespace {

/** Draws a 20-character bar for a level between -90 dBFS and 0 dBFS. */
void buildLevelBar(float levelInDbfs, char* bar, size_t barSize) {
    constexpr int width = 20;
    int filled = static_cast<int>((levelInDbfs + 90.0f) / 90.0f * width + 0.5f);
    if (filled < 0) filled = 0;
    if (filled > width) filled = width;
    size_t i = 0;
    for (; i < static_cast<size_t>(width) && i + 1 < barSize; ++i) {
        bar[i] = static_cast<int>(i) < filled ? '#' : '.';
    }
    bar[i] = '\0';
}

const char* slotName(int slot) {
    return slot == 0 ? "L" : slot == 1 ? "R" : "?";
}

}  // namespace

VoiceNodeDevice::VoiceNodeDevice(const char* deviceIdentifier, Inmp441Microphone& microphoneSensor,
                                 Max98357Speaker& speakerActuator, StreamGatewayClient& edgeLink,
                                 unsigned long schedulerTickInMilliseconds, const VoiceNodeSettings& nodeSettings)
    : Device(schedulerTickInMilliseconds),
      deviceId(deviceIdentifier),
      microphone(microphoneSensor),
      speaker(speakerActuator),
      link(edgeLink),
      settings(nodeSettings),
      tickInMilliseconds(schedulerTickInMilliseconds) {}

bool VoiceNodeDevice::begin() {
    microphone.setGainInDecibels(settings.microphoneGainDb);
    microphone.setVoiceThresholdInDecibels(settings.voiceThresholdDb);
    microphone.setVoiceHangoverInMilliseconds(settings.voiceHangoverMs);
    microphone.setReportIntervalInMilliseconds(settings.levelReportIntervalMs);

    // The frame queue must exist before the capture task starts (the uplink task drains it).
    microphone.enableFrameQueue(settings.micQueueFrames);

    if (!microphone.begin(settings.microphoneChannel)) {
        SerialConsole::printf("[Boot] ERROR: no se pudo iniciar I2S. Revisa los pines y que el core ESP32 sea 3.x.\n");
        return false;
    }
    microphoneReady = true;

    // ModestIoT scheduler: the timer asks the microphone for a measurement every tick.
    microphone.setHandler(this);
    appendSensorToScheduler(&microphone, Sensor::MEASURE_DATA_REQUESTED_EVENT_IDENTIFIER);
    initializeAsynchronousEngine(settings.telemetryQueueLength);

    // Speaker and link: if any fails the microphone keeps working over Serial.
    speaker.setHandler(this);
    speaker.setPlaybackReporter(&link);
    link.setListener(&speaker);
    if (!speaker.begin()) {
        SerialConsole::printf("[Boot] ERROR: no se pudo iniciar el parlante (BCLK=%d LRC=%d DIN=%d).\n",
                              speaker.getBclkPin(), speaker.getLrcPin(), speaker.getDinPin());
    }
    const bool uplinkStarted = xTaskCreatePinnedToCore(&VoiceNodeDevice::uplinkTaskEntry, "MicStream",
                                                       settings.uplinkTaskStackBytes, this,
                                                       settings.uplinkTaskPriority, &uplinkTaskHandle,
                                                       settings.uplinkTaskCore) == pdPASS;
    if (!link.begin() || !uplinkStarted) {
        SerialConsole::printf("[Boot] ERROR: no se pudo iniciar el enlace con el edge.\n");
    }

    char endpoint[96];
    link.describeEndpoint(endpoint, sizeof(endpoint));
    SerialConsole::printf("[Boot] Microfono listo: %lu Hz, 16 bit, mono. SCK=%d WS=%d SD=%d, ganancia %.0f dB\n",
                          static_cast<unsigned long>(microphone.getSampleRate()), microphone.getSckPin(),
                          microphone.getWsPin(), microphone.getSdPin(), microphone.getGainInDecibels());
    SerialConsole::printf("[Boot] Parlante: BCLK=%d LRC=%d DIN=%d, %lu Hz, volumen %u%%. Enlace %s -> %s\n",
                          speaker.getBclkPin(), speaker.getLrcPin(), speaker.getDinPin(),
                          static_cast<unsigned long>(speaker.getSampleRate()),
                          static_cast<unsigned>(speaker.getVolumePercent()), link.transportName(), endpoint);
    printHelp();
    return true;
}

// ---------------------------------------------------------------------------
// ModestIoT event / command / telemetry flow
// ---------------------------------------------------------------------------

void VoiceNodeDevice::on(Event event) {
    TelemetryPackage* package = nullptr;

    if (event.identifier == Inmp441Microphone::SOUND_STARTED_EVENT_IDENTIFIER) {
        package = new VoiceActivityTelemetry(deviceId, true, microphone.getInstantLevelInDbfs());
    } else if (event.identifier == Inmp441Microphone::SOUND_ENDED_EVENT_IDENTIFIER) {
        package = new VoiceActivityTelemetry(deviceId, false, microphone.getInstantLevelInDbfs());
    } else if (event.identifier == Sensor::DATA_READ_EVENT_IDENTIFIER) {
        package = new AudioLevelTelemetry(deviceId, microphone.getLastReport());
    }

    if (package != nullptr && !enqueueTelemetryPayload(&package)) {
        delete package;  // queue full: the NetworkWorker is behind, skip this report
    }
}

void VoiceNodeDevice::handle(Command command) {
    (void)command;  // commands propagated by the speaker (monitoring only)
}

void VoiceNodeDevice::processQueuedTelemetryData(const TelemetryPackage* telemetryPackage) const {
    JsonDocument document;
    telemetryPackage->serialize(document);
    const char* type = document["type"] | "";

    if (strcmp(type, "voice_activity") == 0) {
        const bool active = document["active"] | false;
        const float level = document["levelDbfs"] | -120.0f;
        SerialConsole::printf(active ? "[Mic] >>> Sonido detectado (%.1f dBFS)\n"
                                     : "[Mic] <<< Silencio (%.1f dBFS)\n", level);
    } else if (strcmp(type, "audio_level") == 0 && reportsVisible) {
        const auto* levelPackage = static_cast<const AudioLevelTelemetry*>(telemetryPackage);
        const AudioLevelReport& r = levelPackage->report;
        char bar[24];
        buildLevelBar(r.rmsInDbfs, bar, sizeof(bar));
        SerialConsole::printf(
            "[Mic] %s %6.1f dBFS | pico %6.1f dBFS (%5d) | ruido %6.1f | voz %-2s | clip %lu | "
            "%5.0f Hz | canal %s | gan %.0f dB | perdidas %lu | %s\n",
            bar, r.rmsInDbfs, r.peakInDbfs, r.peakAmplitude, r.noiseFloorInDbfs,
            r.voiceActive ? "SI" : "no", static_cast<unsigned long>(r.clippedSamples),
            r.measuredSampleRateInHertz, slotName(r.activeSlot), r.gainInDecibels,
            static_cast<unsigned long>(r.droppedFrames), Inmp441Microphone::describeStatus(r.status));
        if (r.status != MicrophoneSignalStatus::OK) {
            SerialConsole::printf("      -> %s\n", Inmp441Microphone::describeStatusHint(r.status));
        }
    }

    if (jsonOutputEnabled && (reportsVisible || strcmp(type, "audio_level") != 0)) {
        String json;
        serializeJson(document, json);
        SerialConsole::printf("%s\n", json.c_str());
    }

    const bool isLevel = strcmp(type, "audio_level") == 0;
    if (isLevel) {
        link.addMicQueueDrops(static_cast<const AudioLevelTelemetry*>(telemetryPackage)->report.droppedFrames);
    }
    if (link.isReady() && (!isLevel || settings.sendAudioLevelTelemetry)) {
        String json;
        serializeJson(document, json);
        link.sendText(json.c_str());
    }
}

// ---------------------------------------------------------------------------
// Microphone uplink (core 0, lower priority than the link and the playback)
// ---------------------------------------------------------------------------

void VoiceNodeDevice::uplinkTaskEntry(void* parameters) { static_cast<VoiceNodeDevice*>(parameters)->uplinkLoop(); }

void VoiceNodeDevice::uplinkLoop() {
    constexpr size_t frameSamples = Inmp441Microphone::FRAME_SAMPLES;
    constexpr size_t maxFramesPerMessage = 4;
    const size_t framesPerMessage = settings.framesPerMessage < 1 ? 1
                                  : (settings.framesPerMessage > maxFramesPerMessage ? maxFramesPerMessage
                                                                                      : settings.framesPerMessage);
    static int16_t message[frameSamples * maxFramesPerMessage];
    size_t framesInMessage = 0;

    for (;;) {
        if (!microphone.readFrame(message + framesInMessage * frameSamples, pdMS_TO_TICKS(40))) {
            continue;
        }

        if (!link.isReady()) {
            // No link: the frame is dropped, and so is any half-built message (it would be stale).
            framesInMessage = 0;
            link.countMicFrames(0, 1, 0);
            continue;
        }

        if (++framesInMessage < framesPerMessage) continue;
        framesInMessage = 0;

        if (link.sendAudio(reinterpret_cast<const uint8_t*>(message), framesPerMessage * frameSamples * sizeof(int16_t))) {
            link.countMicFrames(static_cast<uint32_t>(framesPerMessage), 0, 0);
        } else {
            link.countMicFrames(0, 0, static_cast<uint32_t>(framesPerMessage));
        }
    }
}

void VoiceNodeDevice::playTestTone() { speaker.handle(Max98357Speaker::PLAY_TEST_TONE_COMMAND); }

void VoiceNodeDevice::stopPlayback() { speaker.handle(Max98357Speaker::STOP_COMMAND); }
