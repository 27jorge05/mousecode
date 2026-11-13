#ifndef LAZOP_H
#define LAZOP_H

// Controlador proporcional genérico con feedforward y saturación:
//   salida = sat( ff + Kp * (ref - medida) )
// Se usa para distancia (m -> m/s) y para orientación (rad -> rad/s).
class LazoP {
public:
    LazoP(float kp, float salMin, float salMax);

    float calcular(float ref, float medida, float ff = 0.0f);

    void  setKp(float kp);
    void  setLimites(float salMin, float salMax);
    float getKp()    const { return _kp;    }
    float getError() const { return _error; }

private:
    float _kp, _salMin, _salMax;
    float _error;
};

#endif
