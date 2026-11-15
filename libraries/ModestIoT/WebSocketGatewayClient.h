#ifndef WEBSOCKET_GATEWAY_CLIENT_H
#define WEBSOCKET_GATEWAY_CLIENT_H

/**
 * @file WebSocketGatewayClient.h
 * @brief WiFi + WebSocket transport of the voice-edge link (ws://host:port/path).
 * @ingroup network_module
 *
 * Child of StreamGatewayClient. The edge host can be a Windows or a macOS machine: the protocol is the
 * same, only its IP changes (use the PC's LAN IP; a router-reserved IP is recommended).
 *
 * Needs the "WebSockets" library (Markus Sattler / Links2004) 2.7.x.
 */

#include "StreamGatewayClient.h"
#include "WiFiConnectivityDriver.h"

class WebSocketsClient;

class WebSocketGatewayClient : public StreamGatewayClient {
public:
    /**
     * @param wifiDriver WiFi driver (credentials live there).
     * @param edgeHost IP or hostname of the PC running the edge.
     * @param edgePort WebSocket port (default 8765).
     * @param edgePath WebSocket path (default "/ws").
     * @param linkSettings Device id, token, audio format and timings.
     */
    WebSocketGatewayClient(WiFiConnectivityDriver& wifiDriver, const char* edgeHost, uint16_t edgePort,
                           const char* edgePath, const Settings& linkSettings);
    ~WebSocketGatewayClient() override;

    [[nodiscard]] const char* host() const { return edgeHost; }
    [[nodiscard]] uint16_t port() const { return edgePort; }
    [[nodiscard]] const char* path() const { return edgePath; }

    [[nodiscard]] const char* transportName() const override { return "WiFi"; }
    void describeEndpoint(char* destination, size_t capacity) const override;
    void describeCarrier(char* destination, size_t capacity) override;

protected:
    bool transportBegin() override;
    void transportMaintain(unsigned long now) override;
    bool transportCarrierUp() override;
    bool transportOpen() override;
    void transportPump(unsigned long now) override;
    void transportClose() override;
    bool transportSendText(const char* json, size_t length) override;
    bool transportSendAudio(const uint8_t* pcm, size_t length) override;

private:
    void handleSocketEvent(int type, uint8_t* payload, size_t length);

    WiFiConnectivityDriver& wifi;
    const char* edgeHost;
    const uint16_t edgePort;
    const char* edgePath;
    WebSocketsClient* socket = nullptr;  ///< A new client per attempt: no stale library state between reconnects.
    static constexpr unsigned long TCP_PROBE_TIMEOUT_MS = 2000;
};

#endif // WEBSOCKET_GATEWAY_CLIENT_H
