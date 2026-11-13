#include "controlpi.h"

ControlPI::ControlPI(motor &motorRef,
                    float kp, float ki,
                    float ts,
                    float salidaMin, float salidaMax)
    : _motor(motorRef),
    _kp(kp), _ki(ki), _ts(ts),
    _limiteMin(salidaMin), _limiteMax(salidaMax),
    _umbralParada(0.0f),            // antes quedaba sin inicializar
    _zonaMuerta(0.0f),
    _integral(0.0), _salida(0.0), _error(0.0),
    _sentido(1) {}

void ControlPI::configurar(float kp, float ki, float ts) {
    _kp = kp;
    _ki = ki;
    _ts = ts;
}

void ControlPI::setLimites(float salidaMin, float salidaMax) {
    _limiteMin = salidaMin;
    _limiteMax = salidaMax;
}

void ControlPI::reiniciar() {
    _integral = 0.0;
    _salida   = 0.0;
    _error    = 0.0;
}

float ControlPI::_calcularNucleo(float referencia, float medicion) {
    _error = referencia - medicion;

    float integralPropuesta = _integral + _ki * _ts * _error;
    float salidaPropuesta   = _kp * _error + integralPropuesta;

    bool noSaturada        = (salidaPropuesta > _limiteMin && salidaPropuesta < _limiteMax);
    bool satAltaYErrorBaja = (salidaPropuesta >= _limiteMax && _error < 0.0);
    bool satBajaYErrorAlto = (salidaPropuesta <= _limiteMin && _error > 0.0);

    if (noSaturada || satAltaYErrorBaja || satBajaYErrorAlto) {
        _integral = integralPropuesta;
    }

    _salida = _kp * _error + _integral;
    _salida = constrain(_salida, _limiteMin, _limiteMax);
    return _salida;
}

float ControlPI::calcular(float referencia, float medicion) {
    float magRef = fabsf(referencia);
    if (magRef <= 0.0f) {
        _motor.detener();          // IN1=0, IN2=0, EN/PWM=0
        _integral = 0.0;
        _salida   = 0.0;
        _error    = referencia - medicion;   // queda informativo
        return 0.0;
    }

    int8_t sentido = (referencia > 0.0f) ? 1 : -1;
    if (sentido != _sentido) {     // cambio de sentido: no arrastrar el integrador
        _integral = 0.0;
        _sentido  = sentido;
    }

    // El PI ve la velocidad medida en el sentido pedido.
    // Si la rueda aún gira al revés, el error es grande y actúa como freno.
    float pwm = _calcularNucleo(magRef, sentido * medicion);
    if (_zonaMuerta > 0.0f) {                       // salta la zona muerta: 0..100 -> zona..100
        pwm = _zonaMuerta + (100.0f - _zonaMuerta) * pwm / 100.0f;
    }
    if (_umbralParada > 0.0f && pwm < _umbralParada) {
        _motor.detener();
    } else if (sentido > 0) {
        _motor.horario(pwm);
    } else {
        _motor.antihorario(pwm);
    }
    return pwm;
}

float ControlPI::calcularPWM(float referencia, float medicion) {
    return _calcularNucleo(referencia, medicion);
}