#ifndef PWM_H
#define PWM_H

#include <Arduino.h>

class pwm {
    private:
        uint8_t _pin;
    public:
        pwm(uint8_t pin);
        void escribir(uint8_t porcentaje);   // 0-100
        void detener();
};
#endif