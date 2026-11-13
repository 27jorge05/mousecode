#include "codificador.h"

Codificador::Codificador(uint8_t pinA, float ppr, int8_t pinB, bool invertir)
    : _pinA(pinA),
    _pinB(pinB),
    _cuentasPorVuelta(ppr),       // se ajusta en iniciar() si es cuadratura
    _signo(invertir ? -1 : 1),
    _iniciado(false),
    _ultimoConteo(0),
    _ultimoTiempoVel(0),
    _cuentas(0),
    _revoluciones(0.0),
    _grados(0.0) {}

void Codificador::iniciar(uint16_t filtroCiclos) {
    if (_iniciado) return;

    ESP32Encoder::useInternalWeakPullResistors = puType::up;

    if (_pinB >= 0) {
        _enc.attachFullQuad(_pinA, _pinB);
        _cuentasPorVuelta *= 4.0;        // cuadratura x4
    } else {
        _enc.attachSingleEdge(_pinA, -1);
        // _cuentasPorVuelta queda igual (x1)
    }

    _enc.setCount(0);
    _enc.setFilter(filtroCiclos);

    _ultimoConteo    = 0;
    _ultimoTiempoVel = micros();
    _iniciado        = true;
}

void Codificador::reiniciar() {
    if (_iniciado) _enc.setCount(0);

    _ultimoConteo    = 0;
    _ultimoTiempoVel = micros();

    _cuentas      = 0;
    _revoluciones = 0.0;
    _grados       = 0.0;
}

void Codificador::_iniciarsino() {
    if (!_iniciado) iniciar();
}


float Codificador::leerVelocidad() {
    _iniciarsino();

    unsigned long tiempoActual = micros();
    long          conteoActual = _signo * (long)_enc.getCount();

    long  diferenciaCuentas = conteoActual - _ultimoConteo;
    // la resta sin signo es correcta aunque micros() se desborde (~71 min)
    float diferenciaTiempo  = (tiempoActual - _ultimoTiempoVel) / 1000000.0f;  // s

    float velocidad = 0.0;
    if (diferenciaTiempo > 0.0) {
        velocidad = (diferenciaCuentas / _cuentasPorVuelta) / diferenciaTiempo * 60.0;
    }

    _ultimoConteo    = conteoActual;
    _ultimoTiempoVel = tiempoActual;
    return velocidad;
}


void Codificador::leerPosicion() {
    _iniciarsino();

    _cuentas      = _signo * (long)_enc.getCount();
    _revoluciones = _cuentas / _cuentasPorVuelta;   // float por promoción
    _grados       = _revoluciones * 360.0;
}