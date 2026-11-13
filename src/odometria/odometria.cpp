#include "odometria.h"

Odometria::Odometria(float radio, float base)
    : _mPorGrado(radio * PI / 180.0f),
      _base(base),
      _s0(0.0f), _phi0(0.0f),
      _s(0.0f), _phi(0.0f),
      _init(false) {}

void Odometria::actualizar(float gradosL, float gradosR) {
    float sL = gradosL * _mPorGrado;
    float sR = gradosR * _mPorGrado;
    float sAbs   = 0.5f * (sL + sR);
    float phiAbs = (sR - sL) / _base;
    if (!_init) { _s0 = sAbs; _phi0 = phiAbs; _init = true; }
    _s   = sAbs   - _s0;
    _phi = phiAbs - _phi0;
}

void Odometria::reiniciar() {
    _init = false;
}
