/**
 * @file VoiceNodeConsole.cpp
 * @brief Serial console of the voice node: status, link status, live meter, test recordings, WAV dump.
 * @ingroup core_module
 */

#include "VoiceNodeDevice.h"

#include <esp_heap_caps.h>
#include <math.h>
#include "SerialConsole.h"

namespace {

float toDbfs(float amplitude) {
    return amplitude <= 0.0f ? -120.0f : 20.0f * log10f(amplitude / 32768.0f);
}

void writeLittleEndian(uint8_t* destination, uint32_t value, int bytes) {
    for (int i = 0; i < bytes; ++i) destination[i] = static_cast<uint8_t>(value >> (8 * i));
}

}  // namespace

void VoiceNodeDevice::service() {
    if (!microphoneReady) {
        return;
    }
    while (Serial.available() > 0) {
        handleSerialCommand(static_cast<char>(Serial.read()));
    }
    if (liveMeterEnabled && millis() - lastMeterTimestamp >= tickInMilliseconds) {
        lastMeterTimestamp = millis();
        printLiveMeter();
    }
}

// ---------------------------------------------------------------------------
// Recording helpers
// ---------------------------------------------------------------------------

/** Reserves the largest buffer possible up to `seconds` (uses PSRAM when the board has it). */
int16_t* VoiceNodeDevice::allocateRecordingBuffer(uint32_t seconds, size_t& sampleCount) {
    const uint32_t rate = microphone.getSampleRate();
    sampleCount = static_cast<size_t>(seconds) * rate;
    auto* buffer = static_cast<int16_t*>(heap_caps_malloc(sampleCount * sizeof(int16_t), MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT));
    if (buffer != nullptr) return buffer;

    // Without PSRAM the largest free block of internal RAM is usually ~100 KB (~3 s).
    const size_t largestBlock = heap_caps_get_largest_free_block(MALLOC_CAP_8BIT);
    const size_t reserve = 16 * 1024;  // leave room for Serial, JSON and the tasks
    size_t fitting = largestBlock > reserve ? (largestBlock - reserve) / sizeof(int16_t) : 0;
    fitting -= fitting % Inmp441Microphone::FRAME_SAMPLES;
    if (fitting < sampleCount) sampleCount = fitting;
    if (sampleCount < rate / 2) return nullptr;  // less than 0.5 s is useless
    return static_cast<int16_t*>(heap_caps_malloc(sampleCount * sizeof(int16_t), MALLOC_CAP_8BIT));
}

/** Fills `buffer` with fresh microphone audio. Returns the samples recorded. */
size_t VoiceNodeDevice::recordInto(int16_t* buffer, size_t sampleCount) {
    if (!microphone.startRecording(buffer, sampleCount)) return 0;
    const unsigned long timeout = (sampleCount * 1000UL) / microphone.getSampleRate() + 2000;
    const unsigned long start = millis();
    while (microphone.isRecording() && millis() - start < timeout) {
        delay(20);
    }
    const size_t recorded = microphone.getRecordedSampleCount();
    microphone.cancelRecording();
    return recorded;
}

void VoiceNodeDevice::runTestRecording() {
    size_t samples = 0;
    int16_t* buffer = allocateRecordingBuffer(settings.testRecordingSeconds, samples);
    if (buffer == nullptr) {
        SerialConsole::printf("[Prueba] Sin memoria para grabar.\n");
        return;
    }
    SerialConsole::printf("[Prueba] Grabando %.1f s... HABLA AHORA (di \"alivia, prende la luz\").\n",
                          static_cast<float>(samples) / microphone.getSampleRate());
    const size_t recorded = recordInto(buffer, samples);

    int peak = 0;
    uint32_t clipped = 0;
    double sumOfSquares = 0.0;
    double sum = 0.0;
    for (size_t i = 0; i < recorded; ++i) {
        const int value = buffer[i];
        const int magnitude = abs(value);
        if (magnitude > peak) peak = magnitude;
        if (magnitude >= 32767) ++clipped;
        sumOfSquares += static_cast<double>(value) * value;
        sum += value;
    }
    free(buffer);

    if (recorded == 0) {
        SerialConsole::printf("[Prueba] No llego audio. Revisa el estado en las lineas [Mic].\n");
        return;
    }
    const float rms = static_cast<float>(sqrt(sumOfSquares / recorded));
    const float mean = static_cast<float>(sum / recorded);
    const float peakDbfs = toDbfs(static_cast<float>(peak));

    SerialConsole::printf("[Prueba] muestras=%u  pico=%d (%.1f dBFS)  nivel=%.1f dBFS  recortes=%lu  DC=%.1f\n",
                          static_cast<unsigned>(recorded), peak, peakDbfs, toDbfs(rms),
                          static_cast<unsigned long>(clipped), mean);

    if (peak == 0) {
        SerialConsole::printf("[Prueba] Veredicto: SOLO CEROS. Revisa SD, VDD (3.3V), GND y L/R.\n");
    } else if (clipped > recorded / 1000) {
        SerialConsole::printf("[Prueba] Veredicto: RECORTA. Baja la ganancia con '-' (ahora %.0f dB).\n",
                              microphone.getGainInDecibels());
    } else if (peakDbfs < -45.0f) {
        SerialConsole::printf("[Prueba] Veredicto: MUY BAJO. Habla mas cerca o sube la ganancia con '+'.\n");
    } else if (peakDbfs < -25.0f) {
        SerialConsole::printf("[Prueba] Veredicto: BAJO pero usable. Para el edge conviene un pico de -25 a -6 dBFS.\n");
    } else {
        SerialConsole::printf("[Prueba] Veredicto: OK. Buen nivel para Vosk/Whisper.\n");
    }
}

/** Records and sends a WAV over Serial between markers, for tools/grabar_wav.py. */
void VoiceNodeDevice::sendWavRecording() {
    size_t samples = 0;
    int16_t* buffer = allocateRecordingBuffer(settings.wavRecordingSeconds, samples);
    if (buffer == nullptr) {
        SerialConsole::printf("@@ALIVIA_WAV_ERROR sin_memoria\n");
        return;
    }
    const uint32_t rate = microphone.getSampleRate();
    SerialConsole::printf("@@ALIVIA_WAV_RECORDING seconds=%.1f\n", static_cast<float>(samples) / rate);
    const size_t recorded = recordInto(buffer, samples);
    const uint32_t dataBytes = recorded * sizeof(int16_t);

    uint8_t header[44];
    memcpy(header, "RIFF", 4);
    writeLittleEndian(header + 4, 36 + dataBytes, 4);
    memcpy(header + 8, "WAVEfmt ", 8);
    writeLittleEndian(header + 16, 16, 4);        // fmt block size
    writeLittleEndian(header + 20, 1, 2);         // PCM
    writeLittleEndian(header + 22, 1, 2);         // mono
    writeLittleEndian(header + 24, rate, 4);
    writeLittleEndian(header + 28, rate * 2, 4);  // bytes per second
    writeLittleEndian(header + 32, 2, 2);         // bytes per sample
    writeLittleEndian(header + 34, 16, 2);        // bits per sample
    memcpy(header + 36, "data", 4);
    writeLittleEndian(header + 40, dataBytes, 4);

    SerialConsole::beginExclusive();
    Serial.printf("\n@@ALIVIA_WAV_BEGIN size=%lu rate=%lu\n",
                  static_cast<unsigned long>(sizeof(header) + dataBytes), static_cast<unsigned long>(rate));
    Serial.write(header, sizeof(header));
    const auto* bytes = reinterpret_cast<const uint8_t*>(buffer);
    for (uint32_t offset = 0; offset < dataBytes; offset += 512) {
        Serial.write(bytes + offset, min<uint32_t>(512, dataBytes - offset));
    }
    Serial.print("\n@@ALIVIA_WAV_END\n");
    SerialConsole::endExclusive();
    free(buffer);
}

// ---------------------------------------------------------------------------
// Console
// ---------------------------------------------------------------------------

void VoiceNodeDevice::printHelp() {
    // Several short calls: SerialConsole::printf has a 384-byte line buffer.
    SerialConsole::printf("\nComandos:\n  h  ayuda                 s  estado\n  v  vumetro en vivo       j  JSON de telemetria\n");
    SerialConsole::printf("  +  ganancia +3 dB        -  ganancia -3 dB\n  w  estado del enlace     t  tono de prueba (parlante)\n");
    SerialConsole::printf("  r  prueba de %lu s (habla durante la prueba)\n  R  enviar WAV al PC (solo con tools/grabar_wav.py)\n\n",
                          static_cast<unsigned long>(settings.testRecordingSeconds));
}

void VoiceNodeDevice::printStatus() {
    // SerialConsole::printf has a 384-byte line buffer: one short call per line.
    const AudioLevelReport report = microphone.getLastReport();
    SerialConsole::printf("[Estado] pines SCK=%d WS=%d SD=%d | %lu Hz | canal %s | ganancia %.0f dB | estado %s\n",
                          microphone.getSckPin(), microphone.getWsPin(), microphone.getSdPin(),
                          static_cast<unsigned long>(microphone.getSampleRate()),
                          microphone.getActiveSlot() == 0 ? "L" : microphone.getActiveSlot() == 1 ? "R" : "?",
                          microphone.getGainInDecibels(), Inmp441Microphone::describeStatus(report.status));
    SerialConsole::printf("[Estado] %s\n", Inmp441Microphone::describeStatusHint(report.status));
    SerialConsole::printf("[Estado] heap libre %u B, bloque mayor %u B, PSRAM %u B\n",
                          static_cast<unsigned>(ESP.getFreeHeap()),
                          static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)),
                          static_cast<unsigned>(ESP.getFreePsram()));
}

void VoiceNodeDevice::printLinkStatus() {
    const LinkStats stats = link.stats();
    const SpeakerStats sound = speaker.stats();
    char endpoint[96];
    char carrier[96];
    link.describeEndpoint(endpoint, sizeof(endpoint));
    link.describeCarrier(carrier, sizeof(carrier));
    SerialConsole::printf("[Enlace] %s | estado %s | %s | edge %s\n", link.transportName(),
                          StreamGatewayClient::describeState(link.state()), carrier, endpoint);
    SerialConsole::printf("[Enlace] enviado %lu B, recibido %lu B | reconexiones %lu, intentos fallidos %lu, errores de recepcion %lu\n",
                          static_cast<unsigned long>(stats.bytesSent), static_cast<unsigned long>(stats.bytesReceived),
                          static_cast<unsigned long>(stats.reconnects), static_cast<unsigned long>(stats.handshakeFailures),
                          static_cast<unsigned long>(stats.rxErrors));
    SerialConsole::printf("[Enlace] mic: %lu mensajes de audio enviados | descartados sin enlace %lu, fallos de envio %lu, cola del mic %lu\n",
                          static_cast<unsigned long>(stats.micFramesSent), static_cast<unsigned long>(stats.micFramesDroppedNoLink),
                          static_cast<unsigned long>(stats.micFramesDroppedSendFail), static_cast<unsigned long>(stats.micQueueDrops));
    SerialConsole::printf("[Parlante] volumen %u%% | buffer %lu/%lu B | %s | enunciados %lu, underruns %lu, desbordado %lu B, cancelados %lu\n",
                          static_cast<unsigned>(speaker.getVolumePercent()), static_cast<unsigned long>(sound.bufferedBytes),
                          static_cast<unsigned long>(sound.bufferCapacityBytes), sound.playing ? "sonando" : "en silencio",
                          static_cast<unsigned long>(sound.utterances), static_cast<unsigned long>(sound.underruns),
                          static_cast<unsigned long>(sound.overflowBytes), static_cast<unsigned long>(sound.cancelled));
    SerialConsole::printf("[Heap] libre %u B, bloque mayor %u B, minimo historico %u B\n",
                          static_cast<unsigned>(ESP.getFreeHeap()),
                          static_cast<unsigned>(heap_caps_get_largest_free_block(MALLOC_CAP_8BIT)),
                          static_cast<unsigned>(ESP.getMinFreeHeap()));
}

void VoiceNodeDevice::changeGain(float deltaInDecibels) {
    microphone.setGainInDecibels(microphone.getGainInDecibels() + deltaInDecibels);
    SerialConsole::printf("[Mic] Ganancia: %.0f dB\n", microphone.getGainInDecibels());
}

void VoiceNodeDevice::printLiveMeter() {
    const float level = microphone.getInstantLevelInDbfs();
    char bar[41];
    int filled = static_cast<int>((level + 90.0f) / 90.0f * 40.0f + 0.5f);
    filled = constrain(filled, 0, 40);
    for (int i = 0; i < 40; ++i) bar[i] = i < filled ? '#' : '.';
    bar[40] = '\0';
    SerialConsole::printf("[VU] %s %6.1f dBFS %s\n", bar, level, microphone.isVoiceActive() ? "VOZ" : "");
}

void VoiceNodeDevice::handleSerialCommand(char command) {
    switch (command) {
        case 'h': case '?': printHelp(); break;
        case 's': printStatus(); break;
        case 'v':
            liveMeterEnabled = !liveMeterEnabled;
            setReportsVisible(!liveMeterEnabled);
            SerialConsole::printf("[Mic] Vumetro en vivo: %s\n", liveMeterEnabled ? "ON" : "OFF");
            break;
        case 'j':
            setJsonOutputEnabled(!isJsonOutputEnabled());
            SerialConsole::printf("[Mic] JSON: %s\n", isJsonOutputEnabled() ? "ON" : "OFF");
            break;
        case '+': changeGain(3.0f); break;
        case '-': changeGain(-3.0f); break;
        case 'w': printLinkStatus(); break;
        case 't':
            SerialConsole::printf("[Parlante] Tono de prueba: debes oirlo por el parlante.\n");
            playTestTone();
            break;
        case 'r': runTestRecording(); break;
        case 'R': sendWavRecording(); break;
        case '\r': case '\n': case ' ': break;
        default: SerialConsole::printf("[Mic] Comando desconocido '%c'. Escribe h.\n", command); break;
    }
}
