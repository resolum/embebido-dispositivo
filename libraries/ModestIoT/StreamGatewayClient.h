#ifndef STREAM_GATEWAY_CLIENT_H
#define STREAM_GATEWAY_CLIENT_H

/**
 * @file StreamGatewayClient.h
 * @brief Parent class of the real-time audio links to the voice edge (WiFi/WebSocket, Bluetooth/SPP).
 * @ingroup network_module
 *
 * Template Method: this class owns everything that is the same for every transport:
 *   - the link task (core 0): session state machine, retries with exponential backoff,
 *   - the Alivia protocol: hello -> ready/error, audio_begin / PCM / audio_end / audio_cancel,
 *     playback_done, JSON telemetry,
 *   - a mutex that serialises sends from other tasks and fails fast when the link is down,
 *   - counters and the listener (speaker) notifications.
 * Other kinds of nodes (e.g. actuators) plug their own hello and messages in with a LinkProtocolDelegate.
 * Subclasses only implement the transport hooks (transport*): how to open, pump, close and send bytes.
 * The edge side (Windows or macOS) is the same for the ESP32: only the PC's serial-port name differs.
 *
 * It is also a GatewayClient, so it can be used wherever ModestIoT expects one (sendTelemetryRecord).
 */

#include <Arduino.h>
#include "AudioLink.h"
#include "GatewayClient.h"
#include "LinkProtocolDelegate.h"

class StreamGatewayClient : public GatewayClient, public PlaybackReporter {
public:
    struct Settings {
        const char* deviceId = "modest-node";
        const char* mac = "";                   ///< "AA:BB:CC:DD:EE:FF". Empty = the ESP32 base MAC (see baseMacAddress()).
        const char* token = "";
        uint32_t micSampleRate = 16000;
        uint32_t playbackSampleRate = 16000;
        uint16_t frameSamples = 320;
        unsigned long connectTimeoutMs = 5000;  ///< session open + "ready"
        unsigned long backoffMinMs = 1000;
        unsigned long backoffMaxMs = 10000;
        unsigned long pingIntervalMs = 10000;
        unsigned long pongTimeoutMs = 5000;
        uint8_t maxMissedPongs = 3;
        UBaseType_t taskPriority = 3;
        uint32_t taskStackBytes = 8192;
        BaseType_t taskCore = 0;                ///< WiFi/Bluetooth stacks live on core 0
        unsigned long sendLockWaitMs = 15;
    };

    ~StreamGatewayClient() override;

    void setListener(AudioLinkListener* newListener);
    /** @brief Lets another kind of node build its own hello and receive the edge's messages (see LinkProtocolDelegate). */
    void setProtocolDelegate(LinkProtocolDelegate* newDelegate);

    /** @brief Writes the ESP32 base MAC ("AA:BB:CC:DD:EE:FF", 18 bytes with the terminator). Same value the hello carries. */
    static void baseMacAddress(char* destination, size_t capacity);

    /** @brief Starts the transport and the link task. Safe to call once from setup(). */
    bool begin();

    [[nodiscard]] bool isReady() const;
    [[nodiscard]] LinkState state() const;
    static const char* describeState(LinkState state);

    /** @brief Sends one binary audio message. False when the link is not ready or the lock is busy. */
    bool sendAudio(const uint8_t* data, size_t length);
    /** @brief Sends one JSON text message. */
    bool sendText(const char* json);
    bool sendPlaybackDone(const char* utteranceId) override;
    /** @brief GatewayClient contract: serialises the package and sends it as JSON text. */
    bool sendTelemetryRecord(const TelemetryPackage& abstractTelemetryPayload) override;

    void countMicFrames(uint32_t sent, uint32_t droppedNoLink, uint32_t droppedSendFail);
    void addMicQueueDrops(uint32_t drops);
    [[nodiscard]] LinkStats stats() const;
    [[nodiscard]] const Settings& settings() const { return config; }

    /** @brief "WiFi" / "Bluetooth". */
    [[nodiscard]] virtual const char* transportName() const = 0;
    /** @brief Where this node talks to, e.g. "ws://192.168.1.50:8765/ws" or "bluetooth:Alivia-Mic-01". */
    virtual void describeEndpoint(char* destination, size_t capacity) const = 0;
    /** @brief State of the physical link, e.g. "WiFi OK, IP 192.168.1.20, RSSI -52 dBm". */
    virtual void describeCarrier(char* destination, size_t capacity) = 0;

protected:
    StreamGatewayClient(ConnectivityDriver& transportDriver, const char* endpointLabel, const Settings& settings);

    // ---- Transport hooks (implemented by WebSocketGatewayClient / BluetoothStreamGatewayClient) ----
    /** @brief Starts the radio / registers handlers. Called once from begin(), before the task exists. */
    virtual bool transportBegin() = 0;
    /** @brief Called every loop to keep the physical link alive (e.g. WiFi retries). */
    virtual void transportMaintain(unsigned long now) = 0;
    /** @brief true when the physical link is up (WiFi associated / a PC has the serial port open). */
    virtual bool transportCarrierUp() = 0;
    /** @brief Starts a session attempt. Return false (and set failureReason) when it cannot start. No lock held. */
    virtual bool transportOpen() = 0;
    /** @brief Services the session (read bytes, heartbeat). Called with the I/O lock held. */
    virtual void transportPump(unsigned long now) = 0;
    /** @brief Drops the session. Called with the I/O lock held (or from the link task when no I/O is possible). */
    virtual void transportClose() = 0;
    virtual bool transportSendText(const char* json, size_t length) = 0;
    virtual bool transportSendAudio(const uint8_t* pcm, size_t length) = 0;

    // ---- Calls the transport makes into the protocol layer (from transportPump, lock held) ----
    /** @brief The session is open: sends the hello. */
    void onTransportConnected();
    void onTransportText(const uint8_t* payload, size_t length);
    void onTransportAudio(const uint8_t* payload, size_t length);
    /** @brief The peer closed or the session died: the link task tears down and retries with backoff. */
    void onTransportClosed();
    void countReceived(size_t bytes);
    void countRxErrors(uint32_t errors);

    const char* failureReason = "";
    Settings config;

private:
    static void taskEntry(void* parameters);
    void run();
    void tryOpen(unsigned long now);
    void failAttempt(unsigned long now, const char* reason);
    void scheduleRetry(unsigned long now);
    void notifyDown();
    void sendHello();

    AudioLinkListener* listener = nullptr;
    LinkProtocolDelegate* delegate = nullptr;
    SemaphoreHandle_t ioLock = nullptr;
    TaskHandle_t taskHandle = nullptr;

    volatile LinkState linkState = LinkState::CARRIER_DOWN;
    unsigned long stateSince = 0;
    unsigned long nextAttemptAt = 0;
    unsigned long backoffMs = 0;
    volatile bool teardownRequested = false;
    bool announcedDown = true;
    LinkStats counters;
    mutable portMUX_TYPE statsLock = portMUX_INITIALIZER_UNLOCKED;
};

#endif // STREAM_GATEWAY_CLIENT_H
