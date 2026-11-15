/**
 * @file WebSocketGatewayClient.cpp
 * @brief WiFi + WebSocket transport hooks of the voice-edge link.
 * @ingroup network_module
 */

#include "WebSocketGatewayClient.h"

#include <WebSocketsClient.h>
#include <WiFi.h>

WebSocketGatewayClient::WebSocketGatewayClient(WiFiConnectivityDriver& wifiDriver, const char* host,
                                               uint16_t port, const char* path, const Settings& linkSettings)
    : StreamGatewayClient(wifiDriver, path, linkSettings),
      wifi(wifiDriver),
      edgeHost(host),
      edgePort(port),
      edgePath(path) {}

WebSocketGatewayClient::~WebSocketGatewayClient() {
    delete socket;
    socket = nullptr;
}

void WebSocketGatewayClient::describeEndpoint(char* destination, size_t capacity) const {
    snprintf(destination, capacity, "ws://%s:%u%s", edgeHost, static_cast<unsigned>(edgePort), edgePath);
}

void WebSocketGatewayClient::describeCarrier(char* destination, size_t capacity) {
    if (wifi.isConnected()) {
        snprintf(destination, capacity, "WiFi OK, IP %s, RSSI %d dBm", wifi.getLocalAddress().c_str(),
                 wifi.getSignalStrength());
    } else {
        snprintf(destination, capacity, "WiFi sin conexion (%s)", wifi.getSsid());
    }
}

bool WebSocketGatewayClient::transportBegin() {
    wifi.connect();
    return true;
}

void WebSocketGatewayClient::transportMaintain(unsigned long now) {
    (void)now;
    wifi.connect();  // idempotent: only retries after the driver's interval
}

bool WebSocketGatewayClient::transportCarrierUp() { return wifi.isConnected(); }

bool WebSocketGatewayClient::transportOpen() {
    // Probe first: if the PC is unreachable we back off here instead of letting the library
    // retry every 500 ms inside the attempt.
    WiFiClient probe;
    if (!probe.connect(edgeHost, edgePort, TCP_PROBE_TIMEOUT_MS)) {
        failureReason = "el edge no responde por TCP";
        return false;
    }
    probe.stop();

    delete socket;
    socket = new WebSocketsClient();
    socket->begin(edgeHost, edgePort, edgePath, "");  // empty subprotocol: the edge does not negotiate one
    socket->onEvent([this](WStype_t type, uint8_t* payload, size_t length) {
        handleSocketEvent(static_cast<int>(type), payload, length);
    });
    socket->enableHeartbeat(config.pingIntervalMs, config.pongTimeoutMs, config.maxMissedPongs);
    return true;
}

void WebSocketGatewayClient::transportPump(unsigned long now) {
    (void)now;
    if (socket != nullptr) socket->loop();
}

void WebSocketGatewayClient::transportClose() {
    if (socket != nullptr) {
        socket->disconnect();
        delete socket;
        socket = nullptr;
    }
}

bool WebSocketGatewayClient::transportSendText(const char* json, size_t length) {
    return socket != nullptr && socket->sendTXT(json, length);
}

bool WebSocketGatewayClient::transportSendAudio(const uint8_t* pcm, size_t length) {
    return socket != nullptr && socket->sendBIN(pcm, length);
}

void WebSocketGatewayClient::handleSocketEvent(int type, uint8_t* payload, size_t length) {
    // Runs inside socket->loop(), so the I/O lock is already held by the link task.
    switch (static_cast<WStype_t>(type)) {
        case WStype_CONNECTED:
            onTransportConnected();
            break;
        case WStype_TEXT:
            onTransportText(payload, length);
            break;
        case WStype_BIN:
            onTransportAudio(payload, length);
            break;
        case WStype_DISCONNECTED:
            onTransportClosed();
            break;
        default:
            break;  // ping/pong are answered by the library
    }
}
