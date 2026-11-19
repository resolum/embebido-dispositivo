#ifndef LINK_PROTOCOL_DELEGATE_H
#define LINK_PROTOCOL_DELEGATE_H

/**
 * @file LinkProtocolDelegate.h
 * @brief Lets a node that is not a voice node (e.g. the actuators ESP32) speak its own protocol over a
 *        StreamGatewayClient, reusing its transports, retries and send lock.
 * @ingroup network_module
 *
 * Without a delegate the link sends the voice-node hello (microphone + speaker) and routes audio to the listener,
 * exactly as before. With one (StreamGatewayClient::setProtocolDelegate):
 *   - the hello carries type, deviceId, mac and token, and the delegate adds the rest (fillHello),
 *   - the edge's "ready" is handed to onLinkReady, every other JSON message to onText,
 *   - onLinkDown is called when a ready link is lost.
 *
 * The callbacks run in the link task with the I/O lock held: they must be short and must not call sendText()
 * (queue the work and send it from loop()).
 */

#include <ArduinoJson.h>

class LinkProtocolDelegate {
public:
    virtual ~LinkProtocolDelegate() = default;

    /** @brief Adds the node's own fields to the hello (type, deviceId, mac and token are already there). */
    virtual void fillHello(JsonDocument& hello) = 0;
    /** @brief The edge accepted the hello; @p ready is its {"type":"ready",...} answer. */
    virtual void onLinkReady(JsonDocument& ready) {}
    /** @brief A JSON message from the edge other than ready/error, received while the link is ready. */
    virtual void onText(const char* type, JsonDocument& message) {}
    /** @brief The link was ready and is gone. */
    virtual void onLinkDown() {}
};

#endif // LINK_PROTOCOL_DELEGATE_H
