#ifndef MEZCLADOR_H
#define MEZCLADOR_H

#include <Arduino.h>

// Cinemática diferencial inversa: (v, omega) -> referencias de rpm por rueda.
//   v_L = v - omega*base/2 ,  v_R = v + omega*base/2 ,  w = v / c   (c = 2*pi*r/60)
// Cada referencia se limita a [-rpmMax, rpmMax] (el signo indica el sentido de giro de la rueda).
class Mezclador {
public:
    Mezclador(float base, float radio, float rpmMax);

    void mezclar(float v, float omega, float &wL, float &wR) const;

    void setRpmMax(float rpmMax);

private:
    float _base, _c, _rpmMax;
    float _limitar(float w) const;
};

#endif
