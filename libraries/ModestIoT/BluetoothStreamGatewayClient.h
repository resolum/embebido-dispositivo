#ifndef BLUETOOTH_STREAM_GATEWAY_CLIENT_H
#define BLUETOOTH_STREAM_GATEWAY_CLIENT_H

/**
 * @file BluetoothStreamGatewayClient.h
 * @brief Bluetooth Classic (SPP) transport of the voice-edge link, no WiFi and no internet needed.
 * @ingroup network_module
 *
 * Child of StreamGatewayClient. The ESP32 is the SPP server; the edge PC (Windows or macOS) pairs with it
 * and opens the serial port (Windows: the "outgoing" COM port of the pairing; macOS: /dev/cu.<name>).
 * Messages are framed with StreamFrameCodec.h (sync, type, length, CRC16) because a serial stream has no
 * message boundaries. Received bytes are queued by the Bluetooth callback in a ring and decoded in the
 * link task, so the Bluetooth stack task is never blocked.
 *
 * Receive ring: 16 KB by default so the edge may send a 300 ms prefill burst (9.6 KB of PCM) at once.
 *
 * Liveness: the ESP32 sends a PING frame every pingIntervalMs and the edge answers PONG (the edge may
 * also ping; the ESP32 answers PONG). maxMissedPongs silent intervals mark the link as down.
 */

#include "BluetoothConnectivityDriver.h"

#ifdef MODEST_IOT_HAS_CLASSIC_BT

#include "StreamFrameCodec.h"
#include "StreamGatewayClient.h"

class BluetoothStreamGatewayClient : public StreamGatewayClient {
public:
    BluetoothStreamGatewayClient(BluetoothConnectivityDriver& bluetoothDriver, const Settings& linkSettings,
                                 size_t receiveRingBytes = 16384);
    ~BluetoothStreamGatewayClient() override;

    [[nodiscard]] const char* transportName() const override { return "Bluetooth"; }
    void describeEndpoint(char* destination, size_t capacity) const override;
    void describeCarrier(char* destination, size_t capacity) override;

    [[nodiscard]] uint32_t decoderErrors() const;

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
    static void onBluetoothBytes(void* context, const uint8_t* data, size_t length);
    static void onFrame(void* context, uint8_t type, const uint8_t* payload, size_t length);
    void pushReceived(const uint8_t* data, size_t length);
    bool sendFrame(uint8_t type, const uint8_t* payload, size_t length);

    BluetoothConnectivityDriver& bluetooth;

    // Receive ring: written by the Bluetooth stack task, drained by the link task.
    uint8_t* rxRing = nullptr;
    size_t rxCapacity;
    size_t rxHead = 0;
    size_t rxTail = 0;
    size_t rxCount = 0;
    uint32_t rxOverflow = 0;
    mutable portMUX_TYPE rxLock = portMUX_INITIALIZER_UNLOCKED;

    StreamFrame::Decoder decoder;
    uint8_t txFrame[StreamFrame::MAX_FRAME];
    bool helloSent = false;
    unsigned long lastReceivedAt = 0;
    unsigned long lastPingAt = 0;
    uint32_t reportedDecoderErrors = 0;
};

#endif  // MODEST_IOT_HAS_CLASSIC_BT

#endif // BLUETOOTH_STREAM_GATEWAY_CLIENT_H
