#ifndef ALIVIA_BATTERY_MONITOR_H
#define ALIVIA_BATTERY_MONITOR_H

/**
 * @file BatteryMonitor.h
 * @brief Bateria del nodo (Li-ion 1S) leida por un pin ADC con divisor de voltaje, y la telemetria que la informa.
 *
 * Conexion: BAT+ -> R1 -> pin ADC -> R2 -> GND (por ejemplo R1 = R2 = 100 kOhm, ratio 2.0). Usa un pin ADC1
 * (32 a 39) para que funcione con WiFi encendido; 34/35/36/39 son solo de entrada y estan libres en las dos placas.
 *
 * El nodo envia cada minuto {"type":"telemetry","uptimeMs":..,"freeHeap":..,"rssi":..,"batteryMv":..,"batteryPct":..}.
 * El edge avisa al familiar una sola vez cuando batteryPct baja de su umbral (20 % por defecto).
 *
 * Si el pin no tiene bateria (lectura fuera de 2.8 a 4.5 V) la telemetria sale sin batteryMv/batteryPct: un pin al
 * aire nunca dispara un aviso falso. La consola serie permite simular un porcentaje para probar sin bateria.
 *
 * La parte de calculo (percentFromMillivolts, plausible, TelemetryBuilder) no depende del hardware y se prueba en la PC.
 */

#include <stdint.h>
#include <stdio.h>

namespace alivia {

struct BatteryReading {
    bool present = false;  // false: no hay bateria en el pin (o no se pudo leer)
    int millivolts = 0;
    int percent = 0;
};

class BatteryMath {
public:
    static constexpr int MIN_PLAUSIBLE_MV = 2800;
    static constexpr int MAX_PLAUSIBLE_MV = 4500;

    /** Porcentaje aproximado de una celda Li-ion en reposo (curva tipica, interpolada por tramos). */
    static int percentFromMillivolts(int mv) {
        static const int MV[] = {3300, 3400, 3500, 3600, 3700, 3800, 3900, 4000, 4100, 4200};
        static const int PCT[] = {0, 5, 10, 20, 35, 50, 65, 80, 90, 100};
        if (mv <= MV[0]) return 0;
        if (mv >= MV[9]) return 100;
        for (int i = 1; i < 10; i++) {
            if (mv <= MV[i]) return PCT[i - 1] + (mv - MV[i - 1]) * (PCT[i] - PCT[i - 1]) / (MV[i] - MV[i - 1]);
        }
        return 100;
    }

    /** True cuando el voltaje es el de una bateria conectada (y no un pin al aire o en corto). */
    static bool plausible(int mv) { return mv >= MIN_PLAUSIBLE_MV && mv <= MAX_PLAUSIBLE_MV; }
};

/** Arma el JSON de telemetria que entienden los dos edge. Devuelve la longitud escrita (0 si no cupo). */
class TelemetryBuilder {
public:
    static size_t build(char* out, size_t capacity, unsigned long uptimeMs, uint32_t freeHeap, int rssi,
                        const BatteryReading& battery) {
        int n;
        if (battery.present) {
            n = snprintf(out, capacity,
                         "{\"type\":\"telemetry\",\"uptimeMs\":%lu,\"freeHeap\":%lu,\"rssi\":%d,\"batteryMv\":%d,\"batteryPct\":%d}",
                         uptimeMs, (unsigned long)freeHeap, rssi, battery.millivolts, battery.percent);
        } else {
            n = snprintf(out, capacity, "{\"type\":\"telemetry\",\"uptimeMs\":%lu,\"freeHeap\":%lu,\"rssi\":%d}",
                         uptimeMs, (unsigned long)freeHeap, rssi);
        }
        return (n > 0 && (size_t)n < capacity) ? (size_t)n : 0;
    }
};

#ifdef ARDUINO
}  // namespace alivia
#include <Arduino.h>
namespace alivia {

/** Lee la bateria del pin con un promedio de muestras. pin < 0 desactiva la lectura. */
class BatteryMonitor {
public:
    BatteryMonitor(int adcPin, float dividerRatio) : pin(adcPin), ratio(dividerRatio) {}

    void begin() {
        if (pin < 0) return;
        analogSetPinAttenuation(pin, ADC_11db);  // hasta ~3.1 V en el pin
        pinMode(pin, INPUT);
    }

    /** Simula un porcentaje (para probar sin bateria). -1 vuelve a leer el pin. */
    void simulate(int percent) { simulated = percent < 0 ? -1 : (percent > 100 ? 100 : percent); }
    [[nodiscard]] int simulatedPercent() const { return simulated; }

    BatteryReading read() const {
        BatteryReading r;
        if (simulated >= 0) {
            r.present = true;
            r.percent = simulated;
            r.millivolts = 3300 + simulated * 9;  // aproximado, solo informativo
            return r;
        }
        if (pin < 0) return r;
        uint32_t sum = 0;
        for (int i = 0; i < SAMPLES; i++) sum += analogReadMilliVolts(pin);
        const int mv = (int)((float)(sum / SAMPLES) * ratio + 0.5f);
        if (!BatteryMath::plausible(mv)) return r;
        r.present = true;
        r.millivolts = mv;
        r.percent = BatteryMath::percentFromMillivolts(mv);
        return r;
    }

private:
    static constexpr int SAMPLES = 16;
    int pin;
    float ratio;
    int simulated = -1;
};
#endif  // ARDUINO

}  // namespace alivia

#endif  // ALIVIA_BATTERY_MONITOR_H
