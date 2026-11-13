#include "perfil.h"
#include <math.h>

Perfil::Perfil(float vMax, float aMax, float vMin)
    : _vMax(vMax), _aMax(aMax), _vMin(vMin),
      _D(0.0f), _t(0.0f), _ta(0.0f), _tc(0.0f), _tf(0.0f), _vp(0.0f), _da(0.0f),
      _s(0.0f), _v(0.0f), _a(0.0f) {}

void Perfil::iniciar(float D) {
    _D = (D > 0.0f) ? D : 0.0f;
    _t = 0.0f;

    if (_D <= 1e-6f) {                              // nada que recorrer
        _ta = _tc = _tf = 0.0f;
        _vp = _vMin;
        _da = 0.0f;
    } else {
        float daFull = (_vMax * _vMax - _vMin * _vMin) / (2.0f * _aMax);
        if (_D >= 2.0f * daFull) {                  // trapezoidal
            _vp = _vMax;
        } else {                                    // triangular
            _vp = sqrtf(_aMax * _D + _vMin * _vMin);
        }
        _ta = (_vp - _vMin) / _aMax;
        _da = _vMin * _ta + 0.5f * _aMax * _ta * _ta;
        _tc = (_D - 2.0f * _da) / _vp;
        if (_tc < 0.0f) _tc = 0.0f;
        _tf = 2.0f * _ta + _tc;
    }
    _evaluar();
}

void Perfil::actualizar(float dt) {
    _t += dt;
    _evaluar();
}

void Perfil::_evaluar() {
    if (_t < _ta) {                                 // acelerando
        _s = _vMin * _t + 0.5f * _aMax * _t * _t;
        _v = _vMin + _aMax * _t;
        _a = _aMax;
    } else if (_t < _ta + _tc) {                    // crucero
        _s = _da + _vp * (_t - _ta);
        _v = _vp;
        _a = 0.0f;
    } else if (_t < _tf) {                          // frenando
        float td = _tf - _t;
        _s = _D - (_vMin * td + 0.5f * _aMax * td * td);
        _v = _vMin + _aMax * td;
        _a = -_aMax;
    } else {                                        // terminado
        _s = _D;
        _v = _vMin;
        _a = 0.0f;
    }
}
