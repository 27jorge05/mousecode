#ifndef DIGITALPIN_H
#define DIGITALPIN_H

#include <Arduino.h>

class digitalpin {
    private:
        uint8_t _pin;
    public:
        //constructor
        digitalpin(uint8_t pin, uint8_t modo);
    
        void alto();
        void bajo();
        bool leer();    
};
#endif