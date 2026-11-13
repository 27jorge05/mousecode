#include "lazop.h"

LazoP::LazoP(float kp, float salMin, float salMax)
    : _kp(kp), _salMin(salMin), _salMax(salMax), _error(0.0f) {}

float LazoP::calcular(float ref, float medida, float ff) {
    _error = ref - medida;
    float u = ff + _kp * _error;
    if (u < _salMin) u = _salMin;
    if (u > _salMax) u = _salMax;
    return u;
}

void LazoP::setKp(float kp) {
    _kp = kp;
}

void LazoP::setLimites(float salMin, float salMax) {
    _salMin = salMin;
    _salMax = salMax;
}
