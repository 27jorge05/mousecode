#pragma once
#include <Adafruit_VL53L0X.h>
#include <math.h>
#include "SensoresPared.h"

namespace ConfiguracionVL53L0X
{
    // Orden de los arreglos: izquierda (ID 1), frontal (ID 3), derecha (ID 5).
    constexpr int SDA = 25;
    constexpr int SCL = 26;
    constexpr int XSHUT[3] = {32, 27, 14};
    constexpr uint8_t DIRECCIONES[3] = {0x30, 0x31, 0x32};
    constexpr unsigned long PRESUPUESTO_US = 80000;
    struct Coeficientes
    {
        float a, b, c;
    };
    // Primeras tres fórmulas del mensaje: x y resultado en centímetros.
    constexpr Coeficientes AJUSTES[3] = {
        {0.00009978f, 1.018483f, -1.4592f},  // Izquierda
        {-0.00209719f, 1.024291f, -2.9011f}, // Frontal
        {-0.00357056f, 1.024716f, -5.3230f}  // Derecha
    };
    // Valores iniciales para verificar con el montaje real; no son otra calibración.
    constexpr float UMBRALES_CM[3] = {7.0f, 7.0f, 7.0f};
    // El dato anterior solo sirve para decidir en la misma posicion y por 500 ms.
    constexpr unsigned long VIGENCIA_MS = 500;
    constexpr unsigned long ESPERA_MEDICION_MS = 250;
    constexpr unsigned long INTERVALO_SONDEO_MS = 10;
    constexpr float ALPHA_EMA = 0.55f;
    constexpr unsigned long CAPTURA_MS = 5000;
    constexpr float MIN_CRUDO_CM = 5.0f;
    constexpr float MAX_CRUDO_CM = 75.0f;

}

struct LecturaVL53L0X
{
    float crudaCm = NAN;
    float corregidaCm = NAN;
    float filtradaCm = NAN;
    uint8_t estado = 255;
    bool valida = false;
};

// Paredes físicas relativas al carrito, sin consultar PAREDES del simulador.
// Las mediciones se capturan con el robot detenido, antes de decidir un movimiento.
class SensoresParedVL53L0X final : public FuenteParedes
{
public:
    bool calibrar(int pinLed, float referenciaMinCm, float referenciaMaxCm) const;
    bool begin() const override;
    bool actualizar() const override;
    void reiniciarFiltro() const override;
    bool leer(int lado, int fila, int columna, int rumbo) const override;
    bool listo() const;
    bool inicializado() const { return iniciado; }
    bool capturaNueva() const override { return nuevaCaptura; }
    void imprimir() const;
    LecturaVL53L0X lectura(int id) const;
    bool lecturaVigente(int id) const;

private:
    mutable Adafruit_VL53L0X sensores[3];
    mutable LecturaVL53L0X datos[3];
    mutable bool iniciado = false;
    mutable bool pendiente[3] = {};
    mutable unsigned long inicioMedicion[3] = {};
    mutable bool nuevaRonda[3] = {};
    mutable bool lecturaNueva[3] = {};
    mutable bool descartarPendiente[3] = {};
    mutable bool disponiblePosicion[3] = {};
    mutable unsigned long ultimaValida[3] = {};
    mutable bool nuevaCaptura = false;
    mutable unsigned long ultimoSondeo = 0;
    mutable bool capturaValida = false;
    mutable float pendientes[3] = {1, 1, 1};
    mutable float desplazamientos[3] = {};
    mutable bool calibrados[3] = {};
    mutable float ema[3] = {};
    mutable bool emaInicializado[3] = {};
    LecturaVL53L0X medirCrudo(int indice) const;
    LecturaVL53L0X medir(int indice) const;
    bool calibrarSensor(int indice, float realMin, float realMax,
                        float& pendiente, float& desplazamiento) const;
    static void parpadearLed(int pinLed);
    void cargarCalibracion() const;
    bool guardarCalibracion(const float* nuevasPendientes,
                            const float* nuevosDesplazamientos) const;
    static int indice(int id);
};
