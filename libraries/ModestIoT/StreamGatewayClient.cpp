/**
 * @file StreamGatewayClient.cpp
 * @brief Transport-independent part of the audio link (state machine, Alivia protocol, send API).
 * @ingroup network_module
 */

#include "StreamGatewayClient.h"

#include <ArduinoJson.h>
#include <esp_mac.h>
#include "SerialConsole.h"

StreamGatewayClient::StreamGatewayClient(ConnectivityDriver& transportDriver, const char* endpointLabel,
                                         const Settings& settings)
    : GatewayClient(transportDriver, endpointLabel), config(settings), backoffMs(settings.backoffMinMs) {}

StreamGatewayClient::~StreamGatewayClient() {
    if (taskHandle != nullptr) vTaskDelete(taskHandle);
    if (ioLock != nullptr) vSemaphoreDelete(ioLock);
}

void StreamGatewayClient::baseMacAddress(char* destination, size_t capacity) {
    uint8_t mac[6] = {0};
    esp_read_mac(mac, ESP_MAC_WIFI_STA);  // base MAC: available with WiFi off (Bluetooth mode) and equal to WiFi.macAddress()
    snprintf(destination, capacity, "%02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
}

void StreamGatewayClient::setListener(AudioLinkListener* newListener) { listener = newListener; }

bool StreamGatewayClient::begin() {
    if (taskHandle != nullptr) return true;
    ioLock = xSemaphoreCreateMutex();
    if (ioLock == nullptr) return false;
    if (!transportBegin()) return false;
    stateSince = millis();
    return xTaskCreatePinnedToCore(&StreamGatewayClient::taskEntry, "StreamLink", config.taskStackBytes, this,
                                   config.taskPriority, &taskHandle, config.taskCore) == pdPASS;
}

void StreamGatewayClient::taskEntry(void* parameters) { static_cast<StreamGatewayClient*>(parameters)->run(); }

// ---------------------------------------------------------------------------
// Link task
// ---------------------------------------------------------------------------

void StreamGatewayClient::run() {
    for (;;) {
        const unsigned long now = millis();
        transportMaintain(now);

        if (transportCarrierUp()) {
            if (linkState == LinkState::CARRIER_DOWN) {
                linkState = LinkState::DISCONNECTED;
                nextAttemptAt = now;
                char carrier[96];
                describeCarrier(carrier, sizeof(carrier));
                SerialConsole::printf("[Enlace] %s listo: %s\n", transportName(), carrier);
            }
            if (linkState == LinkState::DISCONNECTED && now >= nextAttemptAt) {
                tryOpen(now);
            }
            if (linkState == LinkState::CONNECTING || linkState == LinkState::READY) {
                if (xSemaphoreTake(ioLock, pdMS_TO_TICKS(50)) == pdTRUE) {
                    transportPump(now);
                    xSemaphoreGive(ioLock);
                }
                if (linkState == LinkState::CONNECTING && now - stateSince > config.connectTimeoutMs) {
                    failAttempt(now, "tiempo de espera del handshake");
                }
            }
        } else if (linkState != LinkState::CARRIER_DOWN) {
            SerialConsole::printf("[Enlace] se perdio la conexion %s\n", transportName());
            teardownRequested = true;
        }

        if (teardownRequested) {
            teardownRequested = false;
            xSemaphoreTake(ioLock, portMAX_DELAY);
            transportClose();
            xSemaphoreGive(ioLock);
            notifyDown();
            if (transportCarrierUp()) {
                scheduleRetry(millis());
            } else {
                linkState = LinkState::CARRIER_DOWN;
            }
        }
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

void StreamGatewayClient::tryOpen(unsigned long now) {
    failureReason = "no se pudo abrir la sesion";
    if (!transportOpen()) {
        failAttempt(now, failureReason);
        return;
    }
    linkState = LinkState::CONNECTING;
    stateSince = now;
    portENTER_CRITICAL(&statsLock);
    counters.reconnects++;
    portEXIT_CRITICAL(&statsLock);
    char endpoint[96];
    describeEndpoint(endpoint, sizeof(endpoint));
    SerialConsole::printf("[Enlace] conectando a %s\n", endpoint);
}

void StreamGatewayClient::failAttempt(unsigned long now, const char* reason) {
    SerialConsole::printf("[Enlace] intento fallido (%s); reintento en %lu ms\n", reason, backoffMs);
    portENTER_CRITICAL(&statsLock);
    counters.handshakeFailures++;
    portEXIT_CRITICAL(&statsLock);
    xSemaphoreTake(ioLock, portMAX_DELAY);
    transportClose();
    xSemaphoreGive(ioLock);
    notifyDown();
    scheduleRetry(now);
}

void StreamGatewayClient::scheduleRetry(unsigned long now) {
    linkState = LinkState::DISCONNECTED;
    nextAttemptAt = now + backoffMs;
    backoffMs = min<unsigned long>(backoffMs * 2, config.backoffMaxMs);
}

/** The link was ready and is gone: tell the speaker to cut sound and empty its buffers. */
void StreamGatewayClient::notifyDown() {
    if (announcedDown) return;
    announcedDown = true;
    if (listener != nullptr) listener->onLinkDown();
}

// ---------------------------------------------------------------------------
// Protocol (runs inside transportPump, lock held)
// ---------------------------------------------------------------------------

void StreamGatewayClient::onTransportConnected() { sendHello(); }

void StreamGatewayClient::onTransportClosed() {
    if (linkState != LinkState::CARRIER_DOWN && linkState != LinkState::DISCONNECTED) {
        SerialConsole::printf("[Enlace] conexion cerrada\n");
        teardownRequested = true;
    }
}

void StreamGatewayClient::sendHello() {
    JsonDocument document;
    document["type"] = "hello";
    document["deviceId"] = config.deviceId;
    // The edge identifies the device by its MAC (access table of the backend); deviceId is only a label.
    char mac[18];
    if (config.mac != nullptr && config.mac[0] != '\0') {
        strlcpy(mac, config.mac, sizeof(mac));
    } else {
        baseMacAddress(mac, sizeof(mac));
    }
    document["mac"] = mac;
    JsonArray capabilities = document["capabilities"].to<JsonArray>();
    capabilities.add("microphone");
    capabilities.add("speaker");
    document["micSampleRate"] = config.micSampleRate;
    document["playbackSampleRate"] = config.playbackSampleRate;
    document["bits"] = 16;
    document["channels"] = 1;
    document["frameSamples"] = config.frameSamples;
    document["token"] = config.token;
    String json;
    serializeJson(document, json);
    // The I/O lock is already held by the link task: call the transport directly.
    transportSendText(json.c_str(), json.length());
}

void StreamGatewayClient::onTransportText(const uint8_t* payload, size_t length) {
    countReceived(length);
    JsonDocument document;
    if (deserializeJson(document, payload, length) != DeserializationError::Ok) return;
    const char* type = document["type"] | "";

    if (strcmp(type, "ready") == 0) {
        linkState = LinkState::READY;
        announcedDown = false;
        backoffMs = config.backoffMinMs;
        SerialConsole::printf("[Enlace] enlace listo con el edge (%s)\n", transportName());
    } else if (strcmp(type, "error") == 0) {
        SerialConsole::printf("[Enlace] el edge rechazo el hello: %s\n", document["reason"] | "?");
        teardownRequested = true;  // the teardown applies the backoff
    } else if (listener == nullptr || linkState != LinkState::READY) {
        return;
    } else if (strcmp(type, "audio_begin") == 0) {
        listener->onAudioBegin(document["utteranceId"] | "", document["sampleRate"] | config.playbackSampleRate,
                               strcmp(document["priority"] | "normal", "emergency") == 0);
    } else if (strcmp(type, "audio_end") == 0) {
        listener->onAudioEnd(document["utteranceId"] | "");
    } else if (strcmp(type, "audio_cancel") == 0) {
        listener->onAudioCancel(document["utteranceId"] | "");
    }
}

void StreamGatewayClient::onTransportAudio(const uint8_t* payload, size_t length) {
    countReceived(length);
    if (linkState == LinkState::READY && listener != nullptr) listener->onAudioData(payload, length);
}

// ---------------------------------------------------------------------------
// Send API (other tasks)
// ---------------------------------------------------------------------------

bool StreamGatewayClient::isReady() const { return linkState == LinkState::READY; }

LinkState StreamGatewayClient::state() const { return linkState; }

const char* StreamGatewayClient::describeState(LinkState state) {
    switch (state) {
        case LinkState::CARRIER_DOWN: return "SIN_RED";
        case LinkState::DISCONNECTED: return "DESCONECTADO";
        case LinkState::CONNECTING:   return "CONECTANDO";
        case LinkState::READY:        return "LISTO";
    }
    return "?";
}

bool StreamGatewayClient::sendAudio(const uint8_t* data, size_t length) {
    if (linkState != LinkState::READY || ioLock == nullptr) return false;
    if (xSemaphoreTake(ioLock, pdMS_TO_TICKS(config.sendLockWaitMs)) != pdTRUE) return false;
    const bool sent = linkState == LinkState::READY && transportSendAudio(data, length);
    xSemaphoreGive(ioLock);
    if (sent) {
        portENTER_CRITICAL(&statsLock);
        counters.bytesSent += length;
        portEXIT_CRITICAL(&statsLock);
    }
    return sent;
}

bool StreamGatewayClient::sendText(const char* json) {
    if (linkState != LinkState::READY || ioLock == nullptr || json == nullptr) return false;
    if (xSemaphoreTake(ioLock, pdMS_TO_TICKS(config.sendLockWaitMs)) != pdTRUE) return false;
    const size_t length = strlen(json);
    const bool sent = linkState == LinkState::READY && transportSendText(json, length);
    xSemaphoreGive(ioLock);
    if (sent) {
        portENTER_CRITICAL(&statsLock);
        counters.bytesSent += length;
        portEXIT_CRITICAL(&statsLock);
    }
    return sent;
}

bool StreamGatewayClient::sendPlaybackDone(const char* utteranceId) {
    JsonDocument document;
    document["type"] = "playback_done";
    document["utteranceId"] = utteranceId;
    String json;
    serializeJson(document, json);
    return sendText(json.c_str());
}

bool StreamGatewayClient::sendTelemetryRecord(const TelemetryPackage& abstractTelemetryPayload) {
    const String json = abstractTelemetryPayload.serializeAsString();
    return sendText(json.c_str());
}

// ---------------------------------------------------------------------------
// Counters
// ---------------------------------------------------------------------------

void StreamGatewayClient::countReceived(size_t bytes) {
    portENTER_CRITICAL(&statsLock);
    counters.bytesReceived += static_cast<uint32_t>(bytes);
    portEXIT_CRITICAL(&statsLock);
}

void StreamGatewayClient::countRxErrors(uint32_t errors) {
    portENTER_CRITICAL(&statsLock);
    counters.rxErrors += errors;
    portEXIT_CRITICAL(&statsLock);
}

void StreamGatewayClient::countMicFrames(uint32_t sent, uint32_t droppedNoLink, uint32_t droppedSendFail) {
    portENTER_CRITICAL(&statsLock);
    counters.micFramesSent += sent;
    counters.micFramesDroppedNoLink += droppedNoLink;
    counters.micFramesDroppedSendFail += droppedSendFail;
    portEXIT_CRITICAL(&statsLock);
}

void StreamGatewayClient::addMicQueueDrops(uint32_t drops) {
    portENTER_CRITICAL(&statsLock);
    counters.micQueueDrops += drops;
    portEXIT_CRITICAL(&statsLock);
}

LinkStats StreamGatewayClient::stats() const {
    portENTER_CRITICAL(&statsLock);
    const LinkStats copy = counters;
    portEXIT_CRITICAL(&statsLock);
    return copy;
}
