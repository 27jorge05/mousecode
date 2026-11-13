#pragma once
#include "../mapa/MapaSimulador.h"

// Contrato intercambiable de detección de paredes.
class FuenteParedes {
public:
    virtual ~FuenteParedes();
    virtual bool begin() const = 0;
    // Captura coherente antes de leer. El mapa simulado no necesita hardware.
    virtual bool actualizar() const { return true; }
    virtual bool capturaNueva() const { return true; }
    // Evitar mezclar el EMA de posiciones u orientaciones diferentes.
    virtual void reiniciarFiltro() const {}
    // lado: 0=frente, 1=izquierda, 2=derecha; rumbo: N=0, E=1, S=2, O=3.
    virtual bool leer(int lado, int fila, int columna, int rumbo) const = 0;
};

// Paredes consultadas desde el vector, sin GPIO.
class SensoresParedHardcodeados final : public FuenteParedes {
public:
    bool begin() const override;
    bool leer(int lado, int fila, int columna, int rumbo) const override;
};
