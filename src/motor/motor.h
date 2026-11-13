#ifndef MOTOR_H
#define MOTOR_H

#include "../digitalpin/digitalpin.h"
#include "../pwm/pwm.h"

class motor{
    private: 
        digitalpin _in1;
        digitalpin _in2; 
        pwm        _en;
    public: 
        motor(uint8_t pin1, uint8_t pin2, uint8_t pinPWM);

        void horario(uint8_t porcentaje = 100);
        void antihorario(uint8_t porcentaje = 100);
        void detener();
        void velocidad(uint8_t porcentaje);   // cambia velocidad sin cambiar sentido
};
#endif