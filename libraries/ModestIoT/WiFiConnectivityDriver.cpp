/**
 * @file WiFiConnectivityDriver.cpp
 * @brief Implements the WiFi station connectivity driver.
 * @ingroup network_module
 */

#include <Arduino.h>
#include <WiFi.h>
#include "WiFiConnectivityDriver.h"

WiFiConnectivityDriver::WiFiConnectivityDriver(const char* ssid, const char* password,
                                               unsigned long retryIntervalMs, bool disableModemSleep)
    : networkSsidIdentifier(ssid),
      networkPasswordCredential(password),
      retryIntervalInMilliseconds(retryIntervalMs),
      lowLatency(disableModemSleep),
      provisioningRequested(false)
{
    // Global sketch objects are constructed before the WiFi singleton may be ready: no WiFi.* calls here.
    hardwareSocket = new WiFiClient();
}

WiFiConnectivityDriver::~WiFiConnectivityDriver() {
    if (hardwareSocket != nullptr) {
        delete hardwareSocket;
        hardwareSocket = nullptr;
    }
}

void WiFiConnectivityDriver::connect() {
    if (WiFi.status() == WL_CONNECTED) {
        provisioningRequested = true;
        return;
    }
    const unsigned long now = millis();
    if (!provisioningRequested) {
        WiFi.persistent(false);  // do not write credentials to NVS flash on every connection loop
        WiFi.mode(WIFI_STA);
        if (lowLatency) WiFi.setSleep(false);  // modem sleep adds jitter to the audio
        WiFi.setAutoReconnect(true);
        WiFi.begin(networkSsidIdentifier, networkPasswordCredential);
        provisioningRequested = true;
        lastBeginTimestamp = now;
    } else if (now - lastBeginTimestamp > retryIntervalInMilliseconds) {
        // The first attempt may have failed (wrong password, router rebooting): ask again.
        lastBeginTimestamp = now;
        WiFi.disconnect();
        WiFi.begin(networkSsidIdentifier, networkPasswordCredential);
    }
}

bool WiFiConnectivityDriver::isConnected() {
    return WiFi.status() == WL_CONNECTED;
}

bool WiFiConnectivityDriver::canTransmit() {
    return isConnected();
}

bool WiFiConnectivityDriver::transmit(const char* target, const char* data) {
    if (!isConnected() || target == nullptr || data == nullptr) return false;

    // target = "host" or "host:port" (port defaults to 80).
    String address(target);
    uint16_t port = 80;
    const int colon = address.lastIndexOf(':');
    if (colon > 0) {
        port = static_cast<uint16_t>(address.substring(colon + 1).toInt());
        address = address.substring(0, colon);
    }
    if (!hardwareSocket->connected() && !hardwareSocket->connect(address.c_str(), port, 2000)) {
        return false;
    }
    const size_t length = strlen(data);
    return hardwareSocket->write(reinterpret_cast<const uint8_t*>(data), length) == length;
}

Client& WiFiConnectivityDriver::getNetworkSocket() {
    return *hardwareSocket;
}

int WiFiConnectivityDriver::getSignalStrength() const {
    return WiFi.RSSI();
}

String WiFiConnectivityDriver::getLocalAddress() const {
    return WiFi.localIP().toString();
}
