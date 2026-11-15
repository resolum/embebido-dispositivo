#ifndef WIFI_CONNECTIVITY_DRIVER_H
#define WIFI_CONNECTIVITY_DRIVER_H

/**
 * @file WiFiConnectivityDriver.h
 * @brief Declares the WiFi (station mode) connectivity driver.
 * @ingroup network_module
 *
 * connect() is idempotent and non-blocking: call it as often as you like (the audio link calls it every
 * loop) and it re-issues WiFi.begin() only every `retryIntervalInMilliseconds` while the link is down.
 * Modem sleep is disabled by default because it adds tens of milliseconds of jitter to real-time audio.
 */

#include <WiFi.h>
#include "ConnectivityDriver.h"

class Client;

class WiFiConnectivityDriver : public ConnectivityDriver {
private:
    const char* networkSsidIdentifier;
    const char* networkPasswordCredential;
    unsigned long retryIntervalInMilliseconds;
    bool lowLatency;

    volatile bool provisioningRequested;  ///< WiFi.begin() already issued at least once.
    unsigned long lastBeginTimestamp = 0;

    WiFiClient* hardwareSocket{};

public:
    /**
     * @param ssid Network name.
     * @param password Network password.
     * @param retryIntervalMs Minimum time between WiFi.begin() retries while disconnected.
     * @param disableModemSleep true (default) for low-latency streaming.
     */
    WiFiConnectivityDriver(const char* ssid, const char* password,
                           unsigned long retryIntervalMs = 10000, bool disableModemSleep = true);

    ~WiFiConnectivityDriver() override;

    void connect() override;

    [[nodiscard]] bool isConnected() override;

    [[nodiscard]] bool canTransmit() override;

    /**
     * @brief Sends `data` to `target` ("host:port") over a short TCP connection kept for reuse.
     * @return true when every byte was written.
     */
    bool transmit(const char* target, const char* data) override;

    [[nodiscard]] Client& getNetworkSocket();

    [[nodiscard]] const char* getSsid() const { return networkSsidIdentifier; }
    [[nodiscard]] int getSignalStrength() const;
    [[nodiscard]] String getLocalAddress() const;
};

#endif // WIFI_CONNECTIVITY_DRIVER_H
