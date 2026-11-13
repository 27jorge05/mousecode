#include "digitalpin.h"

digitalpin::digitalpin(uint8_t pin, uint8_t modo){
    _pin = pin;
    pinMode(_pin, modo);
}

void digitalpin::alto(){
    digitalWrite(_pin, HIGH);
}

void digitalpin::bajo(){
    digitalWrite(_pin, LOW);
}

bool digitalpin::leer(){
    return digitalRead(_pin);
}
