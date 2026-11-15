#include "BluetoothConnectivityDriver.h"

#ifdef MODEST_IOT_HAS_CLASSIC_BT

BluetoothConnectivityDriver::BluetoothConnectivityDriver(const char* bluetoothName)
    : deviceName(bluetoothName) {}

void BluetoothConnectivityDriver::setDataHandler(DataHandler handler, void* context) {
    dataHandler = handler;
    dataContext = context;
}

void BluetoothConnectivityDriver::connect() {
    if (started) return;
    if (dataHandler != nullptr) {
        serialBT.onData([this](const uint8_t* buffer, size_t size) {
            if (dataHandler != nullptr) dataHandler(dataContext, buffer, size);
        });
    }
    started = serialBT.begin(deviceName);
}

bool BluetoothConnectivityDriver::isConnected() {
    return started && serialBT.hasClient();
}

bool BluetoothConnectivityDriver::transmit(const char* target, const char* data) {
    (void)target;  // single peer: the target is implicit
    if (!isConnected() || data == nullptr) return false;
    const size_t length = strlen(data);
    return serialBT.write(reinterpret_cast<const uint8_t*>(data), length) == length;
}

bool BluetoothConnectivityDriver::canTransmit() {
    return isConnected();
}

size_t BluetoothConnectivityDriver::write(const uint8_t* data, size_t length) {
    if (!isConnected() || data == nullptr || length == 0) return 0;
    return serialBT.write(data, length);
}

void BluetoothConnectivityDriver::disconnectPeer() {
    if (started) serialBT.disconnect();
}

String BluetoothConnectivityDriver::getAddress() {
    return started ? serialBT.getBtAddressString() : String();
}

#endif  // MODEST_IOT_HAS_CLASSIC_BT
