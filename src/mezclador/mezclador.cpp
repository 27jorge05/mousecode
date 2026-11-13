#include "mezclador.h"

Mezclador::Mezclador(float base, float radio, float rpmMax)
    : _base(base),
      _c(2.0f * PI * radio / 60.0f),
      _rpmMax(rpmMax) {}

void Mezclador::mezclar(float v, float omega, float &wL, float &wR) const {
    float vL = v - omega * _base * 0.5f;
    float vR = v + omega * _base * 0.5f;
    wL = _limitar(vL / _c);
    wR = _limitar(vR / _c);
}

void Mezclador::setRpmMax(float rpmMax) {
    _rpmMax = rpmMax;
}

float Mezclador::_limitar(float w) const {
    if (w < -_rpmMax) return -_rpmMax;
    if (w >  _rpmMax) return  _rpmMax;
    return w;
}
