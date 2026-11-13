#pragma once
#include <Arduino.h>

class IndicadorLed {
public:
    enum class Patron { Apagado, Fijo, Centro, Error, Calibrando };
    explicit IndicadorLed(int pin) : pin(pin) {}
    void begin();
    void actualizar(Patron nuevo, unsigned long ahora);
    // Funcion comun para los dos destellos de la calibracion manual existente.
    static void parpadear(int pin, unsigned int veces, unsigned long encendidoMs,
                         unsigned long apagadoMs);
    static bool nivel(Patron patron, unsigned long transcurrido);
private:
    int pin;
    Patron patron = Patron::Apagado;
    unsigned long inicio = 0;
};
