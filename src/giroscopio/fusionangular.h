#ifndef FUSIONANGULAR_H
#define FUSIONANGULAR_H

#include <Arduino.h>
#include <Wire.h>

// Medición de la orientación angular (yaw, eje Z) del robot.
//
// Combina:
//   - Giroscopio del MPU6050 (integración → phi_imu, dinámica rápida, pero con deriva)
//   - Ángulo phi de la odometría con encoders (absoluto, pero con ruido/slip)
//
// Estrategia: filtro complementario.
//   phi_fusion = alpha * (phi_fusion + dGyro) + (1 - alpha) * phi_enc
//
// Se exponen por separado phi_imu (solo IMU) y phi_fusion para poder comparar
// contra la medición encoder-only que ya entrega Odometria.
//
// El robot debe estar QUIETO durante calibrar().
class FusionAngular {
public:
    FusionAngular(uint8_t sda, uint8_t scl, uint8_t addr = 0x68, float alpha = 0.98f);

    bool begin();
    void calibrar(uint16_t muestras = 500);
    void reiniciar(float phi_inicial = 0.0f);

    // Llamar cada ciclo. dt [s], phi_enc [rad] (odometría).
    void actualizar(float dt, float phi_enc);

    float getPhiFusion() const { return _phiFusion; } // rad, filtro complementario
    float getPhiIMU()    const { return _phiIMU;    } // rad, integración pura del gyro
    float getOmegaZ()    const { return _omegaZ;    } // rad/s, gyro Z crudo
    float getBiasZ()     const { return _biasZ;     } // rad/s
    bool  estaOk()       const { return _ok;        }

    void  setAlpha(float alpha) { _alpha = alpha; }

private:
    TwoWire* _wire;
    uint8_t _sda, _scl, _addr;
    float   _alpha;
    float   _biasZ;
    float   _omegaZ;
    float   _phiIMU;
    float   _phiFusion;
    bool    _ok;

    uint8_t _leerReg(uint8_t reg);
    void    _escribirReg(uint8_t reg, uint8_t val);
    bool    _leerGyroZ(float &gz_rads);
};

#endif