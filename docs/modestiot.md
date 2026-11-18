# ModestIoT en el repo

El firmware usa el framework **ModestIoT** (C++ para ESP32), copiado en `libraries/ModestIoT/`. El sketch
`AliviaMicrophone/AliviaMicrophone.ino` solo declara pines, red y transporte; todo lo demás está en el framework.

## Qué usa el sketch

| Clase | Rol |
|---|---|
| `Inmp441Microphone` (Sensor) | micrófono I2S, captura en el core 1, VAD y niveles |
| `Max98357Speaker` (Actuator) | parlante I2S con jitter buffer y `playback_done` |
| `StreamGatewayClient` | clase padre del enlace: `hello`, backoff, ping, protocolo de audio |
| `WebSocketGatewayClient` | enlace por WiFi (`USE_BLUETOOTH 0`) |
| `BluetoothStreamGatewayClient` | enlace por Bluetooth Classic SPP (`USE_BLUETOOTH 1`) |
| `VoiceNodeDevice` (Device) | une todo: scheduler, telemetría, envío del audio y consola serie |

Los ajustes que antes estaban en `MicrophoneConfig.h` ahora son `VoiceNodeSettings` (ganancia, cola del micrófono,
frames por mensaje) y `SpeakerSettings` (volumen, jitter buffer, prefill). Los valores por defecto no cambian
(ganancia 24 dB, volumen 60 %, cola de 25 frames, 2 frames por mensaje, buffer de 500 ms, prefill de 200 ms).

## Parche local: la MAC en el `hello`

La versión de ModestIoT copiada lleva un cambio que no está en el original, en
`StreamGatewayClient.h` y `StreamGatewayClient.cpp`:

- `Settings::mac`: MAC que se envía en el `hello`. Vacía = la MAC base del ESP32.
- `StreamGatewayClient::baseMacAddress()`: escribe la MAC base (`AA:BB:CC:DD:EE:FF`) leída con `esp_read_mac`, que
  funciona también con el WiFi apagado (modo Bluetooth) y coincide con `WiFi.macAddress()`.
- `sendHello()` añade el campo `mac` (ver [`protocolo-websocket.md`](protocolo-websocket.md)).

Si se actualiza ModestIoT desde su origen, hay que volver a aplicar este cambio o el edge rechazará el `hello`.

## Dependencias

`ModestIoT.h` incluye todos los periféricos del framework, así que el sketch necesita también sus librerías aunque no
los use:

```bash
arduino-cli lib install ArduinoJson WebSockets PubSubClient "DHT sensor library" ESP32Servo "LiquidCrystal I2C" HX711
```

El Arduino IDE solo busca librerías en `~/Documents/Arduino/libraries`, no en el repo. Para que use esta copia, enlázala
(si había otra copia de ModestIoT ahí, muévela fuera de `libraries/` antes):

```bash
ln -s "$PWD/libraries/ModestIoT" ~/Documents/Arduino/libraries/ModestIoT
```

Con `arduino-cli` también sirve `--library libraries/ModestIoT`, que tiene prioridad sobre cualquier otra copia.

## Memoria (compilado con core 3.3.8)

| Variante | Flash | RAM estática |
|---|---|---|
| WiFi, partición `default` | 1 201 199 B (91 % de 1 310 720) | 57 528 B (17 %) |
| Bluetooth, partición `huge_app` | 1 709 856 B (54 % de 3 145 728) | 75 624 B (23 %) |

Con WiFi queda poco margen de flash en la partición `default`; si crece, usar `huge_app`. El heap en ejecución no
está medido en la placa: los comandos `s` y `w` muestran `heap libre` y `bloque mayor`.

## LinkProtocolDelegate (nodos que no son de voz)

`StreamGatewayClient::setProtocolDelegate()` permite que otro tipo de nodo, como el ESP32 de actuadores
(`embebido-actuador`), use el mismo enlace (WiFi/WebSocket o Bluetooth, reintentos y candado de envío) con su propio
protocolo. Con un delegado, el `hello` lleva `type`, `deviceId`, `mac` y `token`, y el delegado agrega el resto
(`fillHello`); el `ready` del edge llega a `onLinkReady`, cualquier otro mensaje a `onText`, y la caída del enlace a
`onLinkDown`. Sin delegado, el enlace se comporta como antes (nodo de voz). Los callbacks corren en la tarea del
enlace con el candado tomado: no deben llamar a `sendText()`.
