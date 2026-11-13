#ifndef ODOMETRIA_H
#define ODOMETRIA_H

#include <Arduino.h>

// Convierte los grados acumulados de cada rueda en distancia y orientación.
//   s   = (sL + sR)/2        [m]   (+ = avanzar)
//   phi = (sR - sL)/base     [rad] (+ = giro a la izquierda, antihorario visto desde arriba)
// Ambas son relativas al origen (la primera lectura, o la última llamada a reiniciar()).
class Odometria {
public:
    Odometria(float radio, float base);

    // Llamar una vez por ciclo con getGrados() de cada encoder.
    void actualizar(float gradosL, float gradosR);

    // El origen pasa a ser la próxima lectura.
    void reiniciar();

    float s()   const { return _s;   }
    float phi() const { return _phi; }

private:
    float _mPorGrado, _base;
    float _s0, _phi0;
    float _s, _phi;
    bool  _init;
};

#endif
