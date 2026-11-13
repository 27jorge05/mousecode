#pragma once
#include "../control/ControlMovimiento.h"
#include "../sensores/SensoresParedVL53L0X.h"

namespace ConfiguracionCalibracionCentro {
    // Experimental: requiere motores habilitados y referencias de pared fiables.
    constexpr bool HABILITADA = false;
    constexpr unsigned long AVISO_CENTRO_MS = 2000;
    constexpr unsigned long ESPERA_LECTURA_MS = 3000;
    constexpr unsigned long DURACION_MAX_MS = 120000;
    constexpr int PASOS_POR_CELDA = 4;
    constexpr float MARGEN_FRONTAL_CM = 1.5f;
    constexpr float MARGEN_LATERAL_CM = 2.0f;
    constexpr float CORRECCION_MIN_GRADOS = 1.0f;
    constexpr float CORRECCION_MAX_GRADOS = 8.0f;
    constexpr float DESACUERDO_MAX_GRADOS = 3.0f;
    constexpr float RESIDUO_MAX_CM = 0.5f;
    constexpr float TOL_RETORNO_CM = 1.5f;
}

// Alineacion angular por paredes, NO calibracion metrica del VL53L0X ni escala gyro.
// Dos vueltas opuestas por el 2x2 central. No modifica el mapa del navegador.
// Solo devuelve exito tras retornar a la celda/rumbo de partida y verificar rangos.
class CalibracionCentro {
public:
    enum class Estado { Inactiva, ReferenciaInicial, Orientar, Medir, Avanzar,
                        Corregir, RestaurarRumbo, VerificarRetorno, Terminada, Error };
    struct AjusteRecta {
        int n = 0;
        float x[5] = {}, y[5] = {};
        void agregar(float distanciaCm, float paredCm);
        bool estimar(float& pendiente, float& residuoCm) const;
    };
    CalibracionCentro(ControlMovimiento& control, SensoresParedVL53L0X& sensores)
        : control(control), sensores(sensores) {}
    bool iniciar(int fila, int columna, int rumbo, int filas, int columnas);
    void actualizar();
    void cancelar();
    Estado estado() const { return actual; }
    bool terminada() const { return actual == Estado::Terminada; }
    bool error() const { return actual == Estado::Error; }
    const char* motivoError() const { return motivo; }
    static float errorAngularGrados(float pendiente, bool izquierda);
private:
    ControlMovimiento& control;
    SensoresParedVL53L0X& sensores;
    Estado actual = Estado::Inactiva;
    const char* motivo = "";
    int origen = 0, celda = 0, rumbo = 0, rumboInicial = 0;
    int vuelta = 0, tramos = 0, paso = 0;
    int referencias[2] = {};
    float inicioDistancia = 0;
    float distanciaAntesPaso = 0;
    float distanciasIniciales[3] = {};
    bool referenciaInicialValida[3] = {};
    unsigned long inicioEspera = 0;
    unsigned long inicioRutina = 0;
    AjusteRecta ajustes[2];
    void esperar(Estado estado);
    bool capturar();
    bool orientar(int destino);
    bool ejecutar(char tipo, float valor);
    void fallar(const char* razon);
};
