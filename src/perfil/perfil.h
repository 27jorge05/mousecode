#ifndef PERFIL_H
#define PERFIL_H

// Planificador trapezoidal de velocidad. Unidades SI (m, m/s, m/s^2).
// Arranca y termina a vMin (no a 0), para no pedir velocidades dentro de la zona muerta del motor.
//   tramo 1: acelera de vMin a vp          (t_a, d_a)
//   tramo 2: crucero a vp                  (t_c)
//   tramo 3: frena de vp a vMin            (t_a, d_a)
// Si la distancia no alcanza para llegar a vMax, el perfil es triangular.
class Perfil {
public:
    Perfil(float vMax, float aMax, float vMin);

    // Planifica una distancia D [m] (>= 0) y reinicia el tiempo.
    void iniciar(float D);

    // Avanza el tiempo interno dt [s] y actualiza s(), v(), a().
    void actualizar(float dt);

    float s() const { return _s; }          // posición de referencia [m]
    float v() const { return _v; }          // velocidad de referencia [m/s]
    float a() const { return _a; }          // aceleración de referencia [m/s^2]
    bool  terminado() const { return _t >= _tf; }

    void  setVmax(float v) { _vMax = v; }
    void  setAmax(float a) { _aMax = a; }
    void  setVmin(float v) { _vMin = v; }
    float getVmin() const  { return _vMin; }
    float getVmax() const  { return _vMax; }

private:
    float _vMax, _aMax, _vMin;
    float _D, _t, _ta, _tc, _tf, _vp, _da;
    float _s, _v, _a;

    void _evaluar();
};

#endif
