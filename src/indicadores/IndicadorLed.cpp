#include "IndicadorLed.h"

void IndicadorLed::begin() {
    pinMode(pin, OUTPUT);
    digitalWrite(pin, LOW);
}

bool IndicadorLed::nivel(Patron patron, unsigned long transcurrido) {
    switch (patron) {
        case Patron::Fijo: return true;
        case Patron::Centro: return transcurrido % 500 < 250;
        case Patron::Calibrando: return transcurrido % 1000 < 100;
        case Patron::Error: {
            // 2 s ON, 0.5 s OFF, 0.5 s ON, 0.5 s OFF; repetir.
            const unsigned long fase = transcurrido % 3500;
            return fase < 2000 || (fase >= 2500 && fase < 3000);
        }
        default: return false;
    }
}

void IndicadorLed::actualizar(Patron nuevo, unsigned long ahora) {
    if (nuevo != patron) { patron = nuevo; inicio = ahora; }
    digitalWrite(pin, nivel(patron, ahora - inicio) ? HIGH : LOW);
}

void IndicadorLed::parpadear(int pin, unsigned int veces,
                            unsigned long encendidoMs, unsigned long apagadoMs) {
    pinMode(pin, OUTPUT);
    for (unsigned int i = 0; i < veces; ++i) {
        digitalWrite(pin, HIGH); delay(encendidoMs);
        digitalWrite(pin, LOW); delay(apagadoMs);
    }
}
