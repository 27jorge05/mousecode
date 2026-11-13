#ifndef CONTROLPI_H
#define CONTROLPI_H

#include <Arduino.h>
#include "../motor/motor.h"

class ControlPI {
public:
    ControlPI(motor &motorRef,
              float kp, float ki,
              float ts        = 0.01,
              float salidaMin = 0.0,
              float salidaMax = 100.0);

    void configurar(float kp, float ki, float ts = 0.01);
    void setLimites(float salidaMin, float salidaMax);
    void setUmbralParada(float pwmMinimo) { _umbralParada = pwmMinimo; }
    // Zona muerta del motor [% PWM]. Si > 0, la salida del PI (0..100) se reescala a
    // [zonaMuerta..100] mientras la referencia sea distinta de 0. 0 = desactivada.
    // Usar ~90-95 % del valor medido para que no avance solo con PI = 0.
    void setZonaMuerta(float pwmZonaMuerta) { _zonaMuerta = pwmZonaMuerta; }
    void reiniciar();

    // Referencia y medición CON SIGNO (rpm de rueda; + = adelante, - = atrás).
    //   ref > 0 -> horario(), ref < 0 -> antihorario(), ref == 0 -> detener().
    // El PI trabaja sobre la magnitud: error = |ref| - (medición en el sentido pedido).
    // Devuelve el PWM aplicado (magnitud, 0..100).
    float calcular(float referencia, float medicion);
    float calcularPWM(float referencia, float medicion);

    float getSalida()   const { return _salida;   }
    float getIntegral() const { return _integral; }
    float getError()    const { return _error;    }

private:
    motor &_motor;
    float  _kp, _ki, _ts;
    float  _limiteMin;          
    float  _limiteMax;
    float  _umbralParada;
    float  _zonaMuerta;
    float  _integral;
    float  _salida;
    float  _error;
    int8_t _sentido;            // último sentido aplicado (+1 / -1)

    float _calcularNucleo(float referencia, float medicion);
};

#endif