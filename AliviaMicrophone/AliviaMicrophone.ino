/**
 * Alivia Voice - ESP32 #1: microfono INMP441 + parlante MAX98357A sobre ModestIoT.
 *
 * Todo lo hace el framework ModestIoT: este sketch solo declara pines, red y transporte.
 *
 *   Inmp441Microphone  (Sensor)    microfono I2S, captura, VAD y niveles
 *   Max98357Speaker    (Actuator)  parlante I2S con jitter buffer
 *   WebSocketGatewayClient         enlace por WiFi   (hijo de StreamGatewayClient)
 *   BluetoothStreamGatewayClient   enlace por Bluetooth Classic (hijo de StreamGatewayClient)
 *   VoiceNodeDevice    (Device)    une todo: scheduler, telemetria, envio del audio y consola serie
 *
 * El audio viaja por el MISMO enlace en los dos sentidos: el microfono sube PCM al edge y el edge devuelve
 * el audio generado, que suena en el parlante. Sin internet: solo WiFi local o Bluetooth.
 *
 * ENLACE CON EL EDGE (lo que debe coincidir, ver docs/protocolo-websocket.md):
 *   - hello = { type, deviceId, mac, capabilities, micSampleRate, playbackSampleRate, bits, channels, frameSamples, token }
 *   - mac   = MAC del ESP32 (AA:BB:CC:DD:EE:FF). Es lo que identifica al dispositivo ante la tabla de acceso del
 *             backend: debe estar registrado como MICROPHONE en estado IN_USE. Se imprime al arrancar.
 *   - token = ALIVIA_WS_TOKEN / [server].token del edge.   - host/puerto/ruta = [server] del edge.
 *   - audio = PCM 16 kHz, 16 bit, mono; tramas de 1280 B (640 muestras, 40 ms).
 *
 * ModestIoT esta copiado en libraries/ModestIoT con el parche de la MAC en StreamGatewayClient (ver docs/modestiot.md).
 *
 * Elegir transporte: cambia USE_BLUETOOTH (0 = WiFi/WebSocket, 1 = Bluetooth).
 *   - WiFi:      el edge corre en una PC (Windows o Mac) en la misma red; EDGE_HOST es la IP de esa PC.
 *   - Bluetooth: empareja la PC con "BT_DEVICE_NAME". Windows: puerto COM saliente del emparejamiento.
 *                Mac: /dev/cu.<BT_DEVICE_NAME>. Placa con "Huge APP" (WiFi y Bluetooth no caben en la
 *                particion por defecto).
 *
 * Requisitos:
 *   - Placa: ESP32 Dev Module (ESP32 clasico). Core "esp32" de Espressif 3.x.
 *   - Librerias: ModestIoT (libraries/ModestIoT del repo, con sus dependencias), ArduinoJson 7.x y WebSockets 2.7.x.
 *   - Monitor serie a 115200 baudios. Comandos: h ayuda, s estado, v vumetro, j JSON, + / - ganancia,
 *     w enlace, t tono de prueba, r prueba de 3 s, R WAV (tools/grabar_wav.py).
 *
 * Sin el dispositivo: tools/simulate_node.py del edge hace de este ESP32 (mismo protocolo).
 */

#include <ModestIoT.h>

// ---------------------------------------------------------------------------
// Configuracion
// ---------------------------------------------------------------------------
#ifndef USE_BLUETOOTH
#define USE_BLUETOOTH 0   // 0 = WiFi + WebSocket, 1 = Bluetooth Classic (SPP)
#endif

static constexpr const char* DEVICE_ID = "alivia-mic-01";   // etiqueta para logs; NO identifica al dispositivo

// Credenciales (WIFI_SSID, WIFI_PASSWORD, EDGE_HOST, EDGE_TOKEN): Secrets.h, ignorado por git.
// Copia Secrets.example.h como Secrets.h y rellenalo. Sin Secrets.h el sketch no compila.
#include "Secrets.h"

static constexpr uint16_t    EDGE_PORT      = 8765;            // [server].port
static constexpr const char* EDGE_PATH      = "/ws";           // [server].path
static constexpr const char* BT_DEVICE_NAME = "Alivia-Mic-01";

// Pines (sin cambios)
static constexpr int MICROPHONE_SCK_PIN = 27;
static constexpr int MICROPHONE_WS_PIN  = 14;
static constexpr int MICROPHONE_SD_PIN  = 2;
static constexpr int SPEAKER_BCLK_PIN   = 26;
static constexpr int SPEAKER_LRC_PIN    = 25;
static constexpr int SPEAKER_DIN_PIN    = 22;

// Audio: debe coincidir con el edge (PCM 16 kHz, 16 bit, mono)
static constexpr uint32_t AUDIO_SAMPLE_RATE = 16000;
static constexpr unsigned long SCHEDULER_TICK_MS = 100;
static constexpr unsigned long SERIAL_BAUD_RATE = 115200;

// ---------------------------------------------------------------------------
// Objetos del framework (el hardware se inicia en setup(), no en los constructores)
// ---------------------------------------------------------------------------
static StreamGatewayClient::Settings linkSettings() {
    StreamGatewayClient::Settings settings;
    settings.deviceId = DEVICE_ID;        // la MAC del hello la agrega StreamGatewayClient
    settings.token = EDGE_TOKEN;
    settings.micSampleRate = AUDIO_SAMPLE_RATE;
    settings.playbackSampleRate = AUDIO_SAMPLE_RATE;
    settings.frameSamples = Inmp441Microphone::FRAME_SAMPLES;
    return settings;
}

#if USE_BLUETOOTH
BluetoothConnectivityDriver connectivity(BT_DEVICE_NAME);
BluetoothStreamGatewayClient edgeLink(connectivity, linkSettings());
#else
WiFiConnectivityDriver connectivity(WIFI_SSID, WIFI_PASSWORD);
WebSocketGatewayClient edgeLink(connectivity, EDGE_HOST, EDGE_PORT, EDGE_PATH, linkSettings());
#endif

Inmp441Microphone microphone(MICROPHONE_SCK_PIN, MICROPHONE_WS_PIN, MICROPHONE_SD_PIN, AUDIO_SAMPLE_RATE);
Max98357Speaker speaker(SPEAKER_BCLK_PIN, SPEAKER_LRC_PIN, SPEAKER_DIN_PIN, AUDIO_SAMPLE_RATE);
VoiceNodeDevice voiceNode(DEVICE_ID, microphone, speaker, edgeLink, SCHEDULER_TICK_MS);

// ---------------------------------------------------------------------------
void setup() {
    SerialConsole::begin(SERIAL_BAUD_RATE);
    delay(300);
    SerialConsole::printf("\n=== Alivia Voice | ESP32 #1 microfono INMP441 + parlante MAX98357A | ModestIoT ===\n");
    char deviceMac[18];
    StreamGatewayClient::baseMacAddress(deviceMac, sizeof(deviceMac));
    SerialConsole::printf("MAC (registrala en el backend como MICROPHONE): %s\n", deviceMac);
#if USE_BLUETOOTH
    SerialConsole::printf("Transporte: Bluetooth SPP como \"%s\" (en el edge: [[bluetooth.links]] mac = \"%s\")\n", BT_DEVICE_NAME, deviceMac);
#else
    SerialConsole::printf("Transporte: WiFi \"%s\" -> ws://%s:%u%s\n", WIFI_SSID, EDGE_HOST, (unsigned)EDGE_PORT, EDGE_PATH);
#endif
    voiceNode.begin();
}

void loop() {
    if (!voiceNode.isReady()) {
        delay(1000);
        return;
    }
    voiceNode.service();
    delay(10);
}
