#ifndef BLUETOOTH_CONNECTIVITY_DRIVER_H
#define BLUETOOTH_CONNECTIVITY_DRIVER_H

/**
 * @file BluetoothConnectivityDriver.h
 * @brief Bluetooth Classic (SPP) connectivity driver, server role: the PC opens the serial port.
 * @ingroup network_module
 *
 * Only available on chips with Classic Bluetooth (ESP32 "classic"); on others
 * MODEST_IOT_HAS_CLASSIC_BT is not defined and the class does not exist.
 *
 * Reception uses BluetoothSerial::onData (the library's own RX queue is only 512 bytes and silently
 * drops data when full), so the driver hands every received chunk to a handler registered with
 * setDataHandler(). The handler runs in the Bluetooth stack task: copy the bytes and return.
 */

#include <Arduino.h>
#include "ConnectivityDriver.h"

#if defined(CONFIG_BT_ENABLED) && defined(CONFIG_BLUEDROID_ENABLED)
#define MODEST_IOT_HAS_CLASSIC_BT 1
#include <BluetoothSerial.h>

class BluetoothConnectivityDriver : public ConnectivityDriver {
public:
    using DataHandler = void (*)(void* context, const uint8_t* data, size_t length);

private:
    BluetoothSerial serialBT;
    const char* deviceName;
    bool started = false;
    DataHandler dataHandler = nullptr;
    void* dataContext = nullptr;

public:
    /** @param bluetoothName Name the PC sees when pairing (also the macOS serial port name). */
    explicit BluetoothConnectivityDriver(const char* bluetoothName = "Modest-IoT-Device");
    ~BluetoothConnectivityDriver() override = default;

    /** @brief Starts the SPP server once (idempotent). */
    void connect() override;
    /** @brief true while a PC has the serial port open. */
    bool isConnected() override;
    bool transmit(const char* target, const char* data) override;
    bool canTransmit() override;

    /** @brief Registers the receive handler. Call before connect(). */
    void setDataHandler(DataHandler handler, void* context);
    /** @brief Writes raw bytes to the peer. Returns the number of bytes accepted (0 on failure). */
    size_t write(const uint8_t* data, size_t length);
    /** @brief Closes the current peer connection. */
    void disconnectPeer();
    [[nodiscard]] const char* getDeviceName() const { return deviceName; }
    [[nodiscard]] String getAddress();
};
#endif  // CONFIG_BT_ENABLED && CONFIG_BLUEDROID_ENABLED

#endif // BLUETOOTH_CONNECTIVITY_DRIVER_H
