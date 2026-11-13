#pragma once

// Coordenadas propias: la salida inicial apunta al norte del mapa aprendido.
// Condiciones fisicas: esquina de un laberinto cuadrado y pared detras al iniciar.
// No utiliza el vector de paredes ni intenta identificar el norte geografico.
class LocalizacionEsquina {
public:
    enum class Estado { SinIniciar, BuscandoInterior, Localizado, Error };
    enum class Interior { Desconocido, Izquierda, Derecha };
    struct Observacion { bool frente, izquierda, derecha; };

    bool iniciar(int filas, int columnas);
    Estado observar(bool frente, bool izquierda, bool derecha);
    bool confirmarAvance();
    Estado estado() const { return estadoActual; }
    Interior interior() const { return ladoInterior; }
    int pasos() const { return avances; }
    int filaInicio() const { return lado - 1; }
    int columnaInicio() const { return ladoInterior == Interior::Derecha ? 0 : lado - 1; }
    int filaActual() const { return filaInicio() - avances; }
    const Observacion& observacion(int paso) const { return historial[paso]; }
    const char* error() const { return motivoError; }

private:
    static constexpr int MAX_LADO = 16;
    Estado estadoActual = Estado::SinIniciar;
    Interior ladoInterior = Interior::Desconocido;
    int lado = 0, avances = 0;
    bool avancePermitido = false;
    Observacion historial[MAX_LADO] = {};
    const char* motivoError = "";
    Estado fallar(const char* motivo);
};
