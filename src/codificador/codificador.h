#ifndef CODIFICADOR_H
#define CODIFICADOR_H

#include <Arduino.h>
#include <ESP32Encoder.h>

class Codificador {
public:
    // pinB >= 0  → cuadratura x4 (attachFullQuad): el conteo es CON SIGNO según el sentido de giro
    // pinB = -1  → 1 canal (attachSingleEdge): solo cuenta hacia arriba, no detecta el sentido
    // ppr        → pulsos por vuelta de RUEDA, por canal (en cuadratura se multiplica x4)
    // invertir   → true si el conteo sale negativo al avanzar (p. ej. motor montado en espejo)
    Codificador(uint8_t pinA,
                float   ppr      = 200.0,
                int8_t  pinB     = -1,
                bool    invertir = false);

    // Configura el hardware (pull-up, canal PCNT, filtro).
    // Se puede llamar varias veces; solo la primera hace efecto.
    // Si no se llama, se auto-invoca en el primer leerVelocidad()/leerPosicion().
    void iniciar(uint16_t filtroCiclos = 100);

    // Reinicia el contador a 0 y limpia los estados internos.
    void reiniciar();

    // --- Mediciones ---
    // Velocidad en RPM (con signo en cuadratura) desde la última llamada.
    // El tiempo se mide con micros().
    // IMPORTANTE: llamarla 1 vez por cada Ts (antes de calcular el error del PI).
    float leerVelocidad();

    // Actualiza los getters de posición (cuentas, revoluciones, grados).
    // No afecta al PI; se puede llamar a otra frecuencia.
    void leerPosicion();

    // --- Getters de posición (tras llamar leerPosicion) ---
    long  getCuentas()      const { return _cuentas;      }
    float getRevoluciones() const { return _revoluciones; }
    float getGrados()       const { return _grados;       }

    // --- Getters de configuración ---
    float getCuentasPorVuelta() const { return _cuentasPorVuelta; }
    bool  estaIniciado()        const { return _iniciado;         }

private:
    ESP32Encoder _enc;
    uint8_t      _pinA;
    int8_t       _pinB;
    float        _cuentasPorVuelta;
    int8_t       _signo;            // +1 normal, -1 si invertir

    bool          _iniciado;
    long          _ultimoConteo;
    unsigned long _ultimoTiempoVel; // [us], de micros()

    long  _cuentas;
    float _revoluciones;
    float _grados;

    void _iniciarsino();
};

#endif