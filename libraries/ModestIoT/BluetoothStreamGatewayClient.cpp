/**
 * @file BluetoothStreamGatewayClient.cpp
 * @brief Bluetooth Classic (SPP) transport hooks of the voice-edge link.
 * @ingroup network_module
 */

#include "BluetoothStreamGatewayClient.h"

#ifdef MODEST_IOT_HAS_CLASSIC_BT

#include <esp_heap_caps.h>

BluetoothStreamGatewayClient::BluetoothStreamGatewayClient(BluetoothConnectivityDriver& bluetoothDriver,
                                                           const Settings& linkSettings,
                                                           size_t receiveRingBytes)
    : StreamGatewayClient(bluetoothDriver, bluetoothDriver.getDeviceName(), linkSettings),
      bluetooth(bluetoothDriver),
      rxCapacity(receiveRingBytes) {}

BluetoothStreamGatewayClient::~BluetoothStreamGatewayClient() {
    if (rxRing != nullptr) heap_caps_free(rxRing);
}

void BluetoothStreamGatewayClient::describeEndpoint(char* destination, size_t capacity) const {
    snprintf(destination, capacity, "bluetooth:%s", bluetooth.getDeviceName());
}

void BluetoothStreamGatewayClient::describeCarrier(char* destination, size_t capacity) {
    snprintf(destination, capacity, "Bluetooth '%s' %s", bluetooth.getDeviceName(),
             bluetooth.isConnected() ? "con la PC conectada" : "esperando a la PC");
}

uint32_t BluetoothStreamGatewayClient::decoderErrors() const {
    return decoder.badHeaders + decoder.crcErrors;
}

bool BluetoothStreamGatewayClient::transportBegin() {
    rxRing = static_cast<uint8_t*>(heap_caps_malloc(rxCapacity, MALLOC_CAP_8BIT));
    if (rxRing == nullptr) return false;
    bluetooth.setDataHandler(&BluetoothStreamGatewayClient::onBluetoothBytes, this);
    bluetooth.connect();
    return true;
}

void BluetoothStreamGatewayClient::transportMaintain(unsigned long now) {
    (void)now;
    bluetooth.connect();  // idempotent
}

bool BluetoothStreamGatewayClient::transportCarrierUp() { return bluetooth.isConnected(); }

bool BluetoothStreamGatewayClient::transportOpen() {
    // There is no handshake below the protocol: the session starts clean and the hello goes out on the first pump.
    portENTER_CRITICAL(&rxLock);
    rxHead = rxTail = rxCount = 0;
    portEXIT_CRITICAL(&rxLock);
    decoder.reset();
    helloSent = false;
    lastReceivedAt = lastPingAt = millis();
    return true;
}

void BluetoothStreamGatewayClient::transportClose() {
    // The serial connection stays open (the PC keeps its port): only the protocol session is reset.
    portENTER_CRITICAL(&rxLock);
    rxHead = rxTail = rxCount = 0;
    portEXIT_CRITICAL(&rxLock);
    decoder.reset();
    helloSent = false;
}

// ---- Reception ------------------------------------------------------------

void BluetoothStreamGatewayClient::onBluetoothBytes(void* context, const uint8_t* data, size_t length) {
    static_cast<BluetoothStreamGatewayClient*>(context)->pushReceived(data, length);
}

/** Runs in the Bluetooth stack task: copy and return. */
void BluetoothStreamGatewayClient::pushReceived(const uint8_t* data, size_t length) {
    if (rxRing == nullptr) return;
    portENTER_CRITICAL(&rxLock);
    const size_t room = rxCapacity - rxCount;
    const size_t accepted = length < room ? length : room;
    for (size_t i = 0; i < accepted; ++i) {
        rxRing[rxHead] = data[i];
        rxHead = (rxHead + 1) % rxCapacity;
    }
    rxCount += accepted;
    rxOverflow += static_cast<uint32_t>(length - accepted);
    portEXIT_CRITICAL(&rxLock);
}

void BluetoothStreamGatewayClient::onFrame(void* context, uint8_t type, const uint8_t* payload, size_t length) {
    auto* self = static_cast<BluetoothStreamGatewayClient*>(context);
    self->lastReceivedAt = millis();
    switch (type) {
        case StreamFrame::TYPE_TEXT:
            self->onTransportText(payload, length);
            break;
        case StreamFrame::TYPE_AUDIO:
            self->onTransportAudio(payload, length);
            break;
        case StreamFrame::TYPE_PING:
            self->sendFrame(StreamFrame::TYPE_PONG, payload, length > 64 ? 64 : length);
            break;
        default:
            break;  // PONG: only refreshes lastReceivedAt
    }
}

void BluetoothStreamGatewayClient::transportPump(unsigned long now) {
    if (!helloSent) {
        helloSent = true;
        onTransportConnected();
    }

    // Drain the ring into the decoder (small chunks keep the spinlock short).
    uint8_t chunk[256];
    for (;;) {
        size_t taken = 0;
        uint32_t overflow = 0;
        portENTER_CRITICAL(&rxLock);
        while (taken < sizeof(chunk) && rxCount > 0) {
            chunk[taken++] = rxRing[rxTail];
            rxTail = (rxTail + 1) % rxCapacity;
            --rxCount;
        }
        overflow = rxOverflow;
        rxOverflow = 0;
        portEXIT_CRITICAL(&rxLock);
        if (overflow > 0) countRxErrors(overflow);
        if (taken == 0) break;
        decoder.feed(chunk, taken, &BluetoothStreamGatewayClient::onFrame, this);
    }

    const uint32_t errors = decoderErrors();
    if (errors != reportedDecoderErrors) {
        countRxErrors(errors - reportedDecoderErrors);
        reportedDecoderErrors = errors;
    }

    // Liveness: ping every interval, give up after maxMissedPongs silent intervals.
    // `now` was read before draining and onFrame() may have stamped a later time: read the clock again
    // (an unsigned subtraction with a stale `now` would wrap to a huge number and close a healthy link).
    now = millis();
    if (now - lastPingAt >= config.pingIntervalMs) {
        lastPingAt = now;
        sendFrame(StreamFrame::TYPE_PING, nullptr, 0);
    }
    if (now - lastReceivedAt > config.pingIntervalMs * config.maxMissedPongs) {
        onTransportClosed();
    }
}

// ---- Sending --------------------------------------------------------------

bool BluetoothStreamGatewayClient::sendFrame(uint8_t type, const uint8_t* payload, size_t length) {
    const size_t size = StreamFrame::encode(type, payload, length, txFrame, sizeof(txFrame));
    if (size == 0) return false;
    return bluetooth.write(txFrame, size) == size;
}

bool BluetoothStreamGatewayClient::transportSendText(const char* json, size_t length) {
    return sendFrame(StreamFrame::TYPE_TEXT, reinterpret_cast<const uint8_t*>(json), length);
}

bool BluetoothStreamGatewayClient::transportSendAudio(const uint8_t* pcm, size_t length) {
    return sendFrame(StreamFrame::TYPE_AUDIO, pcm, length);
}

#endif  // MODEST_IOT_HAS_CLASSIC_BT
