#include "motor.h"

motor::motor(uint8_t pin1, uint8_t pin2, uint8_t pinPWM)
    : _in1(pin1, OUTPUT), _in2(pin2, OUTPUT), _en(pinPWM) {}

void motor::horario(uint8_t porcentaje){
    _in1.alto();
    _in2.bajo();
    _en.escribir(porcentaje);
}

void motor::antihorario(uint8_t porcentaje){
    _in1.bajo();
    _in2.alto();
    _en.escribir(porcentaje);
}

void motor::detener(){
    _in1.bajo();
    _in2.bajo();
    _en.detener();
}

void motor::velocidad(uint8_t porcentaje){
    _en.escribir(porcentaje);
}