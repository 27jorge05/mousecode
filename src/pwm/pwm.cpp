#include "pwm.h"

pwm::pwm(uint8_t pin){
    _pin = pin;
    pinMode(_pin, OUTPUT);
    analogWrite(_pin, 0);
}

void pwm::escribir(uint8_t porcentaje){
    if (porcentaje > 100) porcentaje = 100;      // satura a 100
    uint8_t valor = (uint16_t)porcentaje * 255 / 100;  // 0-100 -> 0-255
    analogWrite(_pin, valor);
}

void pwm::detener(){
    analogWrite(_pin, 0);
}