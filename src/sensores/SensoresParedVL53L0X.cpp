#include "SensoresParedVL53L0X.h"
#include <Wire.h>
#include <math.h>
#include <algorithm>
#include <vector>
#include <Preferences.h>
#include "../indicadores/IndicadorLed.h"

namespace
{
    const char *const NOMBRES[3] = {"IZQ", "FRONTAL", "DER"};
    constexpr int IDS[3] = {1, 3, 5};
    constexpr uint32_t VERSION_CALIBRACION = 1;
    struct CalibracionGuardada {
        uint32_t version;
        int32_t xshut[3];
        float pendientes[3];
        float desplazamientos[3];
    };

    bool registroValido(const CalibracionGuardada& registro) {
        if (registro.version != VERSION_CALIBRACION) return false;
        for (int i = 0; i < 3; ++i) {
            if (registro.xshut[i] != ConfiguracionVL53L0X::XSHUT[i] ||
                !isfinite(registro.pendientes[i]) || registro.pendientes[i] <= 0 ||
                !isfinite(registro.desplazamientos[i])) return false;
        }
        return true;
    }
}

bool SensoresParedVL53L0X::begin() const
{
    using namespace ConfiguracionVL53L0X;
    if (iniciado)
        return true;
    capturaValida = false;
    if (!Wire.begin(
        ConfiguracionVL53L0X::SDA,
        ConfiguracionVL53L0X::SCL,
        400000)) return false;
    Wire.setTimeOut(20); // Limite por transaccion, incluso ante un fallo del bus.

    for (int i = 0; i < 3; ++i)
    {
        pinMode(XSHUT[i], OUTPUT);
        digitalWrite(XSHUT[i], LOW);
    }
    delay(20);
    for (int i = 0; i < 3; ++i)
    {
        digitalWrite(XSHUT[i], HIGH);
        delay(10);
        const bool detectado = sensores[i].begin(DIRECCIONES[i], false, &Wire);
        const bool configurado = detectado && sensores[i].setMeasurementTimingBudgetMicroSeconds(PRESUPUESTO_US);
        if (!configurado)
        {
            Serial.print("ERROR VL53L0X: ");
            Serial.println(NOMBRES[i]);
            for (int j = 0; j < 3; ++j)
                digitalWrite(XSHUT[j], LOW);
            return false;
        }
    }
    cargarCalibracion();
    iniciado = true;
    return true;
}

// Crudo sin polinomio, ajuste ni EMA: se usa durante la calibracion.
LecturaVL53L0X SensoresParedVL53L0X::medirCrudo(int i) const {
    LecturaVL53L0X r;
    if (!pendiente[i]) {
        pendiente[i] = sensores[i].startRange();
        inicioMedicion[i] = millis();
        return r; // La medicion continua en el sensor; no esperar aqui.
    }
    if (!sensores[i].isRangeComplete()) {
        if (millis() - inicioMedicion[i] > ConfiguracionVL53L0X::ESPERA_MEDICION_MS) pendiente[i] = false;
        return r;
    }
    const uint16_t mm = sensores[i].readRangeResult();
    pendiente[i] = false;
    r.estado = sensores[i].readRangeStatus();
    if (descartarPendiente[i]) {
        descartarPendiente[i] = false; // Empezo antes de un cambio de posicion.
        return r;
    }
    r.crudaCm = mm / 10.0f;
    r.valida = r.estado == 0 && mm > 0 && mm != 0xFFFF;
    return r;
}

// Toda lectura normal pasa por el ajuste y despues por el EMA de SU sensor.
LecturaVL53L0X SensoresParedVL53L0X::medir(int i) const {
    LecturaVL53L0X r = medirCrudo(i);
    if (!r.valida) {
        return r;
    }
    const float x = r.crudaCm;
    if (calibrados[i]) {
        r.corregidaCm = pendientes[i] * x + desplazamientos[i];
    } else {
        // Antes de calibrar se conservan tus polinomios originales.
        const auto& c = ConfiguracionVL53L0X::AJUSTES[i];
        r.corregidaCm = (c.a * x + c.b) * x + c.c;
    }
    r.valida = isfinite(r.corregidaCm) && r.corregidaCm >= 0;
    if (!r.valida) {
        return r;
    }
    if (!emaInicializado[i]) {
        ema[i] = r.corregidaCm;
        emaInicializado[i] = true;
    } else {
        const float alpha = ConfiguracionVL53L0X::ALPHA_EMA;
        ema[i] = alpha * r.corregidaCm + (1.0f - alpha) * ema[i];
    }
    r.filtradaCm = ema[i];
    return r;
}

bool SensoresParedVL53L0X::actualizar() const
{
    nuevaCaptura = false;
    for (int i = 0; i < 3; ++i) lecturaNueva[i] = false;
    if (!iniciado) return false;
    const unsigned long ahora = millis();
    // Limitar consultas I2C mientras se siguen atendiendo comandos.
    if (ahora - ultimoSondeo >= ConfiguracionVL53L0X::INTERVALO_SONDEO_MS) {
        ultimoSondeo = ahora;

        for (int i = 0; i < 3; ++i) {
            const LecturaVL53L0X nueva = medir(i);
            if (nueva.valida) {
                datos[i] = nueva;
                lecturaNueva[i] = true;
                disponiblePosicion[i] = true;
                ultimaValida[i] = ahora;
                nuevaRonda[i] = true;
            }
        }
        nuevaCaptura = nuevaRonda[0] && nuevaRonda[1] && nuevaRonda[2];
        if (nuevaCaptura)
            for (int i = 0; i < 3; ++i) nuevaRonda[i] = false;
    }
    capturaValida = true;
    for (int i = 0; i < 3; ++i)
        capturaValida = capturaValida && disponiblePosicion[i] &&
            ahora - ultimaValida[i] <= ConfiguracionVL53L0X::VIGENCIA_MS;
    return capturaValida;
}

void SensoresParedVL53L0X::reiniciarFiltro() const {
    capturaValida = false;
    nuevaCaptura = false;
    for (int i = 0; i < 3; ++i) {
        emaInicializado[i] = false;
        disponiblePosicion[i] = false;
        nuevaRonda[i] = false;
        lecturaNueva[i] = false;
        descartarPendiente[i] = pendiente[i];
        // Conservar datos[i] para mostrarlo, pero no navegar con otra posicion.
    }
}

bool SensoresParedVL53L0X::leer(int lado, int, int, int) const
{
    // actualizar() fija una captura coherente para las tres consultas de pared.
    if (!capturaValida || lado < 0 || lado > 2)
        return true;
    const int i = lado == 0 ? 1 : lado == 1 ? 0
                                            : 2;
    return datos[i].filtradaCm <= ConfiguracionVL53L0X::UMBRALES_CM[i];
}

bool SensoresParedVL53L0X::listo() const
{
    if (!iniciado || !capturaValida) return false;
    const unsigned long ahora = millis();
    for (int i = 0; i < 3; ++i)
        if (!disponiblePosicion[i] || ahora - ultimaValida[i] > ConfiguracionVL53L0X::VIGENCIA_MS) return false;
    return true;
}

int SensoresParedVL53L0X::indice(int id)
{
    if (id == 1)
        return 0;
    if (id == 3)
        return 1;
    if (id == 5)
        return 2;
    return -1;
}

LecturaVL53L0X SensoresParedVL53L0X::lectura(int id) const
{
    const int i = indice(id);
    return i < 0 ? LecturaVL53L0X{} : datos[i];
}

bool SensoresParedVL53L0X::lecturaVigente(int id) const {
    const int i=indice(id);
    return i>=0 && iniciado && disponiblePosicion[i] && datos[i].valida &&
        millis()-ultimaValida[i]<=ConfiguracionVL53L0X::VIGENCIA_MS;
}

void SensoresParedVL53L0X::imprimir() const
{
    for (int i = 0; i < 3; ++i)
    {
        Serial.print("Sensor ");
        Serial.print(IDS[i]);
        Serial.print(" ");
        Serial.print(NOMBRES[i]);
        Serial.print(" XSHUT=D");
        Serial.print(ConfiguracionVL53L0X::XSHUT[i]);
        if (!datos[i].valida) {
            Serial.println(" cruda_cm=--- corregida_cm=--- ema_cm=--- DESCONOCIDO");
            continue;
        }
        Serial.print(" cruda_cm="); Serial.print(datos[i].crudaCm);
        Serial.print(" corregida_cm="); Serial.print(datos[i].corregidaCm);
        Serial.print(" ema_cm="); Serial.print(datos[i].filtradaCm);
        Serial.print(datos[i].filtradaCm <= ConfiguracionVL53L0X::UMBRALES_CM[i] ? " PARED" : " ABERTURA");
        Serial.print(" edad_ms="); Serial.print(millis() - ultimaValida[i]);
        Serial.println(disponiblePosicion[i] && millis() - ultimaValida[i] <= ConfiguracionVL53L0X::VIGENCIA_MS
            ? (lecturaNueva[i] ? " (lectura nueva)" : " (SIN LECTURA NUEVA: conservando anterior)")
            : " (dato anterior: no disponible para navegar)");
    }
}

void SensoresParedVL53L0X::parpadearLed(int pinLed) {
    IndicadorLed::parpadear(pinLed, 2, 200, 200);
}

bool SensoresParedVL53L0X::calibrarSensor(
    int i, float realMin, float realMax, float& m, float& b) const {
    std::vector<float> muestras;
    muestras.reserve(128);
    const unsigned long inicio = millis();
    while (millis() - inicio < ConfiguracionVL53L0X::CAPTURA_MS) {
        // Durante la captura se descarta texto; ! cancela.
        while (Serial.available()) {
            if (Serial.read() == '!') {
                Serial.println("Calibracion cancelada.");
                return false;
            }
        }
        const LecturaVL53L0X r = medirCrudo(i);
        if (r.valida) muestras.push_back(r.crudaCm);
        delay(1);
    }
    if (muestras.size() < 10) {
        Serial.println("Captura rechazada: menos de 10 lecturas validas.");
        return false;
    }
    std::sort(muestras.begin(), muestras.end());
    // Igual que tu codigo: extremos robustos P05 y P95.
    const float minimo = muestras[(muestras.size() - 1) * 5 / 100];
    const float maximo = muestras[(muestras.size() - 1) * 95 / 100];
    if (maximo - minimo < 0.1f) {
        Serial.println("Captura rechazada: no hubo suficiente variacion.");
        return false;
    }
    m = (realMax - realMin) / (maximo - minimo);
    b = realMin - m * minimo;
    if (!isfinite(m) || !isfinite(b) || m <= 0) return false;
    Serial.print("Sensor "); Serial.print(IDS[i]);
    Serial.print(" P05_cm="); Serial.print(minimo);
    Serial.print(" P95_cm="); Serial.print(maximo);
    Serial.print(" pendiente="); Serial.print(m);
    Serial.print(" desplazamiento_cm="); Serial.println(b);
    return true;
}

bool SensoresParedVL53L0X::calibrar(
    int pinLed, float realMin, float realMax) const {
    if (!isfinite(realMin) || !isfinite(realMax) ||
        realMin < 0 || realMax <= realMin) {
        Serial.println("Referencias invalidas.");
        return false;
    }
    if (!begin()) return false;
    reiniciarFiltro();
    float nuevasPendientes[3], nuevosDesplazamientos[3];
    for (int i = 0; i < 3; ++i) {
        Serial.print("Preparar "); Serial.print(NOMBRES[i]);
        Serial.print(". Barrer entre "); Serial.print(realMin);
        Serial.print(" y "); Serial.print(realMax); Serial.println(" cm.");
        parpadearLed(pinLed);
        Serial.println("CAPTURANDO 5 segundos...");
        if (!calibrarSensor(i, realMin, realMax,
                            nuevasPendientes[i], nuevosDesplazamientos[i])) {
            Serial.println("No se aplicaron cambios: se conserva el ajuste anterior.");
            return false;
        }
    }
    // Un solo registro para guardar juntos los ajustes de los tres sensores.
    // Si falla la escritura, conservar tambien en RAM la calibracion anterior.
    if (!guardarCalibracion(nuevasPendientes, nuevosDesplazamientos)) {
        Serial.println("ERROR: no se pudo guardar en flash; se conserva el ajuste anterior.");
        return false;
    }
    // Aplicar solo cuando las tres capturas y el guardado terminaron correctamente.
    for (int i = 0; i < 3; ++i) {
        pendientes[i] = nuevasPendientes[i];
        desplazamientos[i] = nuevosDesplazamientos[i];
        calibrados[i] = true;
        emaInicializado[i] = false;
        datos[i] = LecturaVL53L0X{};
    }
    Serial.println("Calibracion guardada en flash. Se conserva al apagar y se carga al arrancar.");
    return true;
}

bool SensoresParedVL53L0X::guardarCalibracion(
    const float* nuevasPendientes, const float* nuevosDesplazamientos) const {
    CalibracionGuardada registro = {};
    registro.version = VERSION_CALIBRACION;
    for (int i = 0; i < 3; ++i) {
        registro.xshut[i] = ConfiguracionVL53L0X::XSHUT[i];
        registro.pendientes[i] = nuevasPendientes[i];
        registro.desplazamientos[i] = nuevosDesplazamientos[i];
    }
    if (!registroValido(registro)) return false;
    Preferences memoria;
    if (!memoria.begin("mouse-tof", false)) return false;
    const bool ok = memoria.putBytes("calibracion", &registro, sizeof(registro)) == sizeof(registro);
    memoria.end();
    return ok;
}

void SensoresParedVL53L0X::cargarCalibracion() const {
    Preferences memoria;
    if (!memoria.begin("mouse-tof", true)) {
        Serial.println("Sin calibracion guardada disponible: se usan los polinomios originales.");
        return;
    }
    CalibracionGuardada registro = {};
    const size_t leidos = memoria.getBytes("calibracion", &registro, sizeof(registro));
    memoria.end();
    if (leidos != sizeof(registro) || !registroValido(registro)) {
        Serial.println("Sin calibracion guardada compatible: se usan los polinomios originales.");
        return;
    }
    for (int i = 0; i < 3; ++i) {
        pendientes[i] = registro.pendientes[i];
        desplazamientos[i] = registro.desplazamientos[i];
        calibrados[i] = true;
        emaInicializado[i] = false;
    }
    Serial.println("Calibracion cargada desde flash: IZQ / FRONTAL / DER.");
}
