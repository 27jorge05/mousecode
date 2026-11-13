#include <Arduino.h>
#include <string.h>
#include <ctype.h>
#include "algoritmos/CiclicoRecto.h"
#include "sensores/SensoresParedVL53L0X.h"
#include <stdio.h>
#include <Wire.h>
#include "indicadores/IndicadorLed.h"
#include "calibracion/CalibracionCentro.h"
// #include "algoritmos/CiclicoCurvo.h"

// Botón físico de arranque (GPIO 0 es el botón BOOT integrado en las placas ESP32)
constexpr int PIN_BOTON = 0;

// LED integrado en la placa ESP32 (GPIO 2)
constexpr int PIN_LED = 2;
IndicadorLed indicadorLed(PIN_LED);
CalibracionCentro* calibracionCentro = nullptr;
enum class EstadoCentro { Inactivo, Avisando, Calibrando };
EstadoCentro estadoCentro = EstadoCentro::Inactivo;
unsigned long inicioAvisoCentro = 0;
int llegadasNotificadas = 0;

// Robot físico: control recibido y paredes medidas con tres VL53L0X a 90 grados.
ControlMovimiento *movimiento = nullptr;
CiclicoRecto *navegador = nullptr;
// CiclicoCurvo* navegador = nullptr;
// Antes: SensoresParedHardcodeados* paredes = nullptr; // Paredes del vector del mapa.
SensoresParedVL53L0X *paredes = nullptr;
FuenteParedes *fuenteParedes = nullptr;
constexpr unsigned long BAUDIOS = 921600;
constexpr unsigned long INTERVALO_ESTADO_MS = 5000;
const char *etapaArranque = "INICIALIZANDO";
unsigned long ultimoEstadoMs = 0;
unsigned long ultimaLecturaMs = 0;
constexpr unsigned long INTERVALO_SENSORES_MS = 500;
bool preparado = false;
bool activo = false;
bool mostrarSensores = false;
bool arranqueUsado = false;
constexpr bool USAR_MAPA_SIMULADO = false;
char comando[32] = {};
uint8_t longitudComando = 0;
bool comandoInvalido = false;


// Prueba aislada: no crea objetos de control ni usa la libreria de la IMU/ToF.
void escanearI2C()
{
    const auto imprimirHex = [](uint8_t valor) {
        char texto[5];
        snprintf(texto, sizeof(texto), "0x%02X", static_cast<unsigned int>(valor));
        Serial.print(texto);
    };
    const auto sondear = [](uint8_t direccion) {
        Wire.beginTransmission(direccion);
        return Wire.endTransmission(true);
    };
    Serial.println("=== PRUEBA AISLADA I2C ===");
    Serial.print("SDA=D"); Serial.print(ConfiguracionVL53L0X::SDA);
    Serial.print(" SCL=D"); Serial.println(ConfiguracionVL53L0X::SCL);
    Serial.println("Sin inicializar motores, encoders, algoritmos ni calibracion.");

    // Evitar que los tres VL53L0X respondan simultaneamente en 0x29.
    for (int i = 0; i < 3; ++i) {
        pinMode(ConfiguracionVL53L0X::XSHUT[i], OUTPUT);
        digitalWrite(ConfiguracionVL53L0X::XSHUT[i], LOW);
    }
    delay(20);
    if (!Wire.begin(ConfiguracionVL53L0X::SDA, ConfiguracionVL53L0X::SCL, 100000)) {
        Serial.println("ERROR: no se pudo iniciar el controlador I2C.");
        return;
    }
    Serial.println("Controlador I2C iniciado a 100 kHz.");
    int encontrados = 0;
    int timeouts = 0;
    for (uint8_t direccion = 0x08; direccion <= 0x77; ++direccion) {
        const uint8_t resultado = sondear(direccion);
        if (resultado == 5) ++timeouts;
        if (resultado != 0) continue;
        Serial.print("Responde: "); imprimirHex(direccion); Serial.println();
        ++encontrados;
    }
    Serial.print("Direcciones que responden con ToF apagados: "); Serial.println(encontrados);
    Serial.print("Sondeos con timeout: "); Serial.println(timeouts);
    bool imuIdentificada = false;
    for (uint8_t direccion : {uint8_t(0x68), uint8_t(0x69)}) {
        const uint8_t resultado = sondear(direccion);
        Serial.print("MPU "); imprimirHex(direccion);
        Serial.print(" resultado="); Serial.println(resultado);
        if (resultado != 0) continue;
        Wire.beginTransmission(direccion);
        Wire.write(uint8_t(0x75));
        if (Wire.endTransmission(false) != 0) {
            Serial.println("ERROR preparando lectura WHO_AM_I.");
            continue;
        }
        const size_t recibidos = Wire.requestFrom(direccion, uint8_t(1));
        Serial.print("Bytes recibidos: "); Serial.println(recibidos);
        if (recibidos != 1) continue;
        const uint8_t who = Wire.read();
        Serial.print("WHO_AM_I: "); imprimirHex(who); Serial.println();
        if (who == 0x68) {
            imuIdentificada = true;
            Serial.println("OK: MPU6050 identificado mediante lectura real.");
        } else Serial.println("Responde, pero la identidad no corresponde al MPU6050 esperado.");
    }
    const char* nombres[3] = {"IZQUIERDO", "FRONTAL", "DERECHO"};
    int tofDetectados = 0;
    for (int i = 0; i < 3; ++i) {
        digitalWrite(ConfiguracionVL53L0X::XSHUT[i], HIGH);
        delay(20);
        const uint8_t resultado = sondear(0x29);
        Serial.print("ToF "); Serial.print(nombres[i]);
        Serial.print(" XSHUT=D"); Serial.print(ConfiguracionVL53L0X::XSHUT[i]);
        Serial.print(" direccion=0x29 resultado="); Serial.println(resultado);
        if (resultado == 0) ++tofDetectados;
        digitalWrite(ConfiguracionVL53L0X::XSHUT[i], LOW);
        delay(20);
    }
    Serial.print("MPU identificado: "); Serial.println(imuIdentificada ? "SI" : "NO");
    Serial.print("Sensores ToF que responden: "); Serial.println(tofDetectados);
    Serial.print("Nivel SDA en reposo: "); Serial.println(digitalRead(ConfiguracionVL53L0X::SDA));
    Serial.print("Nivel SCL en reposo: "); Serial.println(digitalRead(ConfiguracionVL53L0X::SCL));
    Serial.println("Resultado del sondeo: 0=respuesta, 2=NACK direccion, 5=timeout; otros=error.");
    Serial.println("Fin. No se inicia el robot. Pulsar RESET para repetir.");
    Serial.println("ToF apagados. Esta prueba verifica comunicacion, no mide distancias ni valida todo el hardware.");
}


void imprimirAyuda()
{
    // Serial.println("FAB: monitor a 921600 baudios, enviar con Nueva linea (LF/CR/CRLF).");
    Serial.println("========== COMANDOS ==========");

    Serial.println("INICIAR");
    Serial.println("  Inicia el intento con el algoritmo seleccionado.");
    Serial.println("  En modo sin motores solo muestra la primera accion.");

    Serial.println("ESTADO");
    Serial.println("  Muestra la etapa actual y el estado del recorrido.");

    Serial.println("SENSORES");
    Serial.println("  Muestra una lectura puntual de los tres sensores.");
    Serial.println("  Incluye distancia cruda, corregida y filtrada por EMA.");

    Serial.println("MOSTRARSENSORES");
    Serial.println("  Activa o desactiva la visualizacion periodica.");
    Serial.println("  Envia el mismo comando otra vez para cambiar el estado.");

    Serial.println("CALIBRAR minimo_cm maximo_cm");
    Serial.println("  Ejemplo: CALIBRAR 0 10");
    Serial.println("  Usa las distancias reales de referencia indicadas.");
    Serial.println("  Orden: izquierdo, frontal y derecho.");
    Serial.println("  Dos parpadeos antes de cada captura de 5 segundos.");
    Serial.println("  Mueve una superficie entre las referencias frente al sensor.");
    Serial.println("  Si las tres capturas son validas, guarda en flash.");
    Serial.println("  La calibracion se recupera al volver a encender.");
    Serial.println("  Ejecutar antes de INICIAR.");

    Serial.println("MAPA");
    Serial.println("  Muestra dimensiones, tamano de casilla, inicio y ciclos.");
    Serial.println("  La fuente depende de USAR_MAPA_SIMULADO; con sensores se localiza la esquina.");

    Serial.println("AYUDA");
    Serial.println("  Vuelve a mostrar esta lista.");

    Serial.println("!");
    Serial.println("  Fuera de calibracion: detiene y bloquea hasta reiniciar.");
    Serial.println("  Durante la captura de calibracion: cancela el ajuste.");

    Serial.println("LED centro: parpadeo rapido. Error: 2 s ON, 0.5 OFF, 0.5 ON, 0.5 OFF.");
    Serial.println(ConfiguracionCalibracionCentro::HABILITADA
        ? "Calibracion de paralelismo en centro: ACTIVADA (experimental)."
        : "Calibracion de paralelismo en centro: DESACTIVADA en CalibracionCentro.h.");
    Serial.println("Enviar comandos con Enter / nueva linea.");
    Serial.println("==============================");
}

void imprimirEstado()
{
    Serial.print("FAB ESTADO: ");
    Serial.print(etapaArranque);
    Serial.print(" | preparado=");
    Serial.print(preparado);
    Serial.print(" | activo=");
    Serial.print(activo);
    Serial.print(" | arranque_usado=");
    Serial.print(arranqueUsado);
    Serial.print(" | fallo_control=");
    Serial.println(movimiento ? movimiento->fallo : false);
    Serial.print(" | sensores_validos=");
    Serial.println(USAR_MAPA_SIMULADO ? preparado : (paredes && paredes->listo()));
    Serial.print(" | centro_localizado=");
    Serial.println(navegador && navegador->centroLocalizado());
    if (preparado && !activo && !arranqueUsado)
        Serial.println("Esperando INICIAR + Enter o BOOT. No se mueve automaticamente.");
}

// Mantener visible el estado aunque el monitor se abra después de setup().
void informarEspera()
{
    if (activo || millis() - ultimoEstadoMs < INTERVALO_ESTADO_MS)
        return;
    ultimoEstadoMs = millis();
    imprimirEstado();
}

void normalizarComando()
{
    unsigned int principio = 0;
    while (principio < longitudComando && isspace(static_cast<unsigned char>(comando[principio])))
        ++principio;
    unsigned int fin = longitudComando;
    while (fin > principio && isspace(static_cast<unsigned char>(comando[fin - 1])))
        --fin;
    longitudComando = fin - principio;
    memmove(comando, comando + principio, longitudComando);
    for (unsigned int i = 0; i < longitudComando; ++i)
        comando[i] = static_cast<char>(toupper(static_cast<unsigned char>(comando[i])));
    comando[longitudComando] = '\0';
}

void actualizarLED()
{
    using Patron = IndicadorLed::Patron;
    Patron patron = Patron::Apagado;
    const bool error = (movimiento && movimiento->fallo) ||
        strncmp(etapaArranque, "ERROR", 5) == 0;
    if (error) patron = Patron::Error;
    else if (estadoCentro == EstadoCentro::Calibrando) patron = Patron::Calibrando;
    else if (estadoCentro == EstadoCentro::Avisando ||
             strcmp(etapaArranque, "FINALIZADO_EN_CENTRO") == 0) patron = Patron::Centro;
    else if (preparado && navegador && ((!activo && !arranqueUsado) || navegador->enInicio()))
        patron = Patron::Fijo;
    indicadorLed.actualizar(patron, millis());
}

void finalizarRecorrido()
{
    activo = false;
    const bool enCentro = navegador->esMeta(navegador->celdaActual());
    etapaArranque = enCentro ? "FINALIZADO_EN_CENTRO" : "DETENIDO_SIN_META";
    movimiento->stop();
    Serial.print(enCentro ? "FINALIZADO EN EL CENTRO. Llegadas: " : "DETENIDO SIN META. Llegadas: ");
    Serial.println(navegador->carrerasCompletadas);
    Serial.println("Reiniciar ESP32 para repetir desde la posicion inicial.");
    actualizarLED();
}

void notificarLlegadaCentro()
{
    if (!navegador->centroLocalizado() || !navegador->esMeta(navegador->celdaActual()) ||
        navegador->carrerasCompletadas <= llegadasNotificadas) return;
    llegadasNotificadas = navegador->carrerasCompletadas;
    movimiento->stop();
    estadoCentro = EstadoCentro::Avisando;
    inicioAvisoCentro = millis();
    etapaArranque = "EN_CENTRO";
    Serial.print("CENTRO ALCANZADO. Llegada "); Serial.println(llegadasNotificadas);
    Serial.println("LED parpadeando. Pausa de 2 segundos antes de continuar.");
    actualizarLED();
}

bool atenderCentro()
{
    if (estadoCentro == EstadoCentro::Inactivo) return false;
    if (estadoCentro == EstadoCentro::Avisando) {
        if (millis()-inicioAvisoCentro < ConfiguracionCalibracionCentro::AVISO_CENTRO_MS) return true;
        if (ConfiguracionCalibracionCentro::HABILITADA && paredes && calibracionCentro) {
            estadoCentro = EstadoCentro::Calibrando;
            etapaArranque = "CALIBRACION_CENTRO";
            calibracionCentro->iniciar(navegador->filaActual(), navegador->columnaActual(),
                navegador->direccionActual(), navegador->filas, navegador->columnas);
        } else {
            estadoCentro = EstadoCentro::Inactivo;
            if (navegador->fase == 3) finalizarRecorrido();
            else etapaArranque = "EJECUTANDO";
        }
        return true;
    }
    calibracionCentro->actualizar();
    if (calibracionCentro->error()) {
        movimiento->stop(); movimiento->fallo = true;
        activo = preparado = false;
        estadoCentro = EstadoCentro::Inactivo;
        etapaArranque = "ERROR_CALIBRACION_CENTRO";
        actualizarLED();
    } else if (calibracionCentro->terminada()) {
        estadoCentro = EstadoCentro::Inactivo;
        if (navegador->fase == 3) finalizarRecorrido();
        else etapaArranque = "EJECUTANDO";
    }
    return true;
}

void imprimirMapa()
{
    Serial.println(USAR_MAPA_SIMULADO ? "=== MAPA SIMULADO ===" : "=== SENSORES REALES / LOCALIZACION AUTOMATICA ===");
    Serial.print("Filas: "); Serial.print(MapaSimulador::FILAS);
    Serial.print(" Columnas: "); Serial.println(MapaSimulador::COLUMNAS);
    Serial.print("Casilla (cm): "); Serial.println(MapaSimulador::CASILLA_CM);
    Serial.print("Ciclos: "); Serial.println(ConfiguracionUnificada::CICLOS);
    if (USAR_MAPA_SIMULADO) {
        Serial.print("Inicio fila,columna,rumbo: "); Serial.print(MapaSimulador::INICIO_FILA);
        Serial.print(","); Serial.print(MapaSimulador::INICIO_COLUMNA);
        Serial.print(","); Serial.println(MapaSimulador::INICIO_RUMBO);
        for (int r = 0; r < MapaSimulador::FILAS; ++r) {
            for (int c = 0; c < MapaSimulador::COLUMNAS; ++c) {
                Serial.print(MapaSimulador::PAREDES[r * MapaSimulador::COLUMNAS + c]);
                Serial.print(" ");
            }
            Serial.println();
        }
    } else {
        Serial.println("No se consulta el vector PAREDES ni la esquina configurada del simulador.");
        Serial.println("Colocar en una esquina con pared detras y una salida libre al frente.");
        if (navegador && navegador->centroLocalizado()) {
            Serial.print("Inicio propio fila,columna: "); Serial.print(navegador->filaInicial());
            Serial.print(","); Serial.println(navegador->columnaInicial());
            Serial.print("Posicion propia fila,columna: "); Serial.print(navegador->filaActual());
            Serial.print(","); Serial.println(navegador->columnaActual());
        } else Serial.println("Inicio y centro: pendientes de localizacion al enviar INICIAR.");
    }
}

void imprimirSensores()
{
    if (activo)
    {
        Serial.println("SENSORES solo se consulta con el carrito detenido.");
        return;
    }
    if (!paredes) { Serial.println("Modo mapa: no hay sensores fisicos seleccionados."); return; }
    if (!paredes->begin())
    {
        Serial.println("ERROR: sensores no inicializados.");
        return;
    }
    const bool validos = paredes->actualizar();
    paredes->imprimir();
    if (!validos)
        Serial.println("Hay lecturas desconocidas; revisar montaje, calibracion y rango.");
}

// Distancias visibles continuamente, incluso tras detener una orden de prueba.
void mostrarDistanciasPeriodicamente()
{
    if (!mostrarSensores || !paredes || millis() - ultimaLecturaMs < INTERVALO_SENSORES_MS)
        return;
    ultimaLecturaMs = millis();
    if (!paredes->begin())
        return;
    paredes->actualizar();
    paredes->imprimir();
}

void abortar()
{
    if (!movimiento)
        return;
    movimiento->fallo = true;
    movimiento->stop();
    activo = false;
    preparado = false;
    if (estadoCentro == EstadoCentro::Calibrando && calibracionCentro) calibracionCentro->cancelar();
    estadoCentro = EstadoCentro::Inactivo;
    etapaArranque = "ABORTADO";
    actualizarLED();
    Serial.println("ABORTADO. Motores detenidos. Reiniciar ESP32 para otra prueba.");
}

void iniciarRecorrido(const char *origen)
{
    if (!preparado || arranqueUsado || !fuenteParedes || !navegador || !movimiento || movimiento->fallo)
    {
        Serial.println("No se puede iniciar. Estado actual:");
        imprimirEstado();
        return;
    }
    arranqueUsado = true;
    etapaArranque = "EJECUTANDO";
    Serial.print("INICIO (");
    Serial.print(origen);
    
    if (strcmp(origen, "BOTON FISICO") == 0)
    {
        Serial.println("Esperando 1 segundo para retirar la mano del robot...");
        delay(1000);
    }
    navegador->begin();
    activo = !movimiento->fallo && navegador->fase != 3;
    if (!activo)
    {
        etapaArranque = "FALLO_AL_INICIAR";
        movimiento->stop();
        imprimirEstado();
    }
    actualizarLED();
}

void atenderBoton()
{
    if (activo || arranqueUsado || !preparado)
        return;

    static bool botonPrevio = HIGH;
    static unsigned long tCambio = 0;

    bool lectura = digitalRead(PIN_BOTON);
    if (lectura != botonPrevio && (millis() - tCambio > 50))
    {
        tCambio = millis();
        botonPrevio = lectura;
        if (lectura == LOW)
        { // Pulsado (con INPUT_PULLUP)
            iniciarRecorrido("BOTON FISICO");
        }
    }
}
void calibrarSensores(float minimoCm, float maximoCm)
{
    if (activo || arranqueUsado || !paredes || !movimiento)
    {
        Serial.println("Calibrar antes de INICIAR; reiniciar si ya hubo un intento.");
        return;
    }
    movimiento->stop();
    etapaArranque = "CALIBRANDO";
    const bool ok = paredes->calibrar(PIN_LED, minimoCm, maximoCm);
    paredes->actualizar();
    preparado = paredes->inicializado() && !movimiento->fallo;
    paredes->imprimir();
    etapaArranque = preparado ? "LISTO" : "ERROR_VL53L0X";
    Serial.println(ok ? "Calibracion terminada." : "Calibracion no aplicada.");
    actualizarLED();
}

void atenderSerial()
{
    while (Serial.available())
    {
        const int value = Serial.read();
        if (value < 0)
            return;
        const char ch = static_cast<char>(value);
        if (ch == '!')
        {
            longitudComando = 0;
            comandoInvalido = false;
            abortar();
            return;
        }
        if (ch == '\r' || ch == '\n')
        {
            if (comandoInvalido)
            {
                Serial.println("Comando demasiado largo; descartado. Usar AYUDA.");
                comandoInvalido = false;
                longitudComando = 0;
                continue;
            }
            if (longitudComando == 0)
                continue;
            normalizarComando();
            if (longitudComando == 0)
                continue;
            if (strcmp(comando, "MAPA") == 0)
                imprimirMapa();
            else if (strcmp(comando, "SENSORES") == 0)
                imprimirSensores();
            else if (strcmp(comando, "MOSTRARSENSORES") == 0)
            {
                mostrarSensores = !mostrarSensores;

                Serial.println(mostrarSensores
                                   ? "Visualizacion de sensores ACTIVADA"
                                   : "Visualizacion de sensores DESACTIVADA");
            }
            else if (strncmp(comando, "CALIBRAR", 8) == 0)
            {
                float minimoCm = 0, maximoCm = 0;
                char extra = 0;
                if (sscanf(comando, "CALIBRAR %f %f %c",
                           &minimoCm, &maximoCm, &extra) == 2)
                {
                    calibrarSensores(minimoCm, maximoCm);
                }
                else
                {
                    Serial.println("Usar CALIBRAR minimo_cm maximo_cm; ejemplo: CALIBRAR 3 10");
                }
            }
            else if (strcmp(comando, "ESTADO") == 0)
                imprimirEstado();
            else if (strcmp(comando, "AYUDA") == 0)
                imprimirAyuda();
            else if (strcmp(comando, "INICIAR") == 0)
                iniciarRecorrido("SERIAL");
            else
            {
                Serial.print("Comando desconocido: ");
                Serial.println(comando);
                imprimirAyuda();
            }
            longitudComando = 0;
        }
        else if (!comandoInvalido)
        {

            if (static_cast<unsigned int>(longitudComando) + 1u < sizeof(comando))
                comando[longitudComando++] = ch;
            else
                comandoInvalido = true;
        }
    }
}

void setup()
{
    Serial.begin(BAUDIOS);
    pinMode(PIN_BOTON, INPUT_PULLUP);
    indicadorLed.begin();
    delay(500); // Permitir que el monitor reciba el inicio del diagnostico.
    // escanearI2C();
    // Inicializar GPIO después del arranque de Arduino, no como objetos globales.
    Serial.println("FAB: construyendo control y navegador...");
    static ControlMovimiento control;
    
    // static SensoresParedHardcodeados sensorMapa;
    // static CiclicoRecto algoritmo(control,sensorMapa);
    // movimiento=&control; paredes=&sensorMapa; navegador=&algoritmo;

    static SensoresParedVL53L0X sensores;
    static CalibracionCentro alineacion(control, sensores);
    calibracionCentro = &alineacion;
    static SensoresParedHardcodeados sensorMapa;
    FuenteParedes& origenParedes = USAR_MAPA_SIMULADO
        ? static_cast<FuenteParedes&>(sensorMapa)
        : static_cast<FuenteParedes&>(sensores);
    static CiclicoRecto algoritmo(control, origenParedes, !USAR_MAPA_SIMULADO);
    // static CiclicoCurvo algoritmo(control, origenParedes, !USAR_MAPA_SIMULADO);
    movimiento = &control;
    paredes = USAR_MAPA_SIMULADO ? nullptr : &sensores;
    fuenteParedes = &origenParedes;
    navegador = &algoritmo;
    control.stop();
    delay(200);


    imprimirAyuda();
    imprimirMapa();
    Serial.println(" INICIAR muestra la primera orden y detiene el intento.");

    
    etapaArranque = "INICIALIZANDO_CONTROL_IMU";
    Serial.println("Mantener quieto: calibrando MPU6050...");
    if (!control.begin())
    {
        etapaArranque = "ERROR_IMU_CONTROL";
        Serial.println("ERROR IMU/control: no se permite iniciar.");
        control.stop();
        return;
    }
    
    etapaArranque = USAR_MAPA_SIMULADO ? "VALIDANDO_MAPA" : "INICIALIZANDO_VL53L0X";
    if (!fuenteParedes->begin()) {
        etapaArranque = USAR_MAPA_SIMULADO ? "ERROR_MAPA" : "ERROR_VL53L0X";
        Serial.println("ERROR: no se pudo inicializar la fuente de paredes seleccionada.");
        control.stop();
        return;
    }
    const bool lecturaInicial = fuenteParedes->actualizar();
    if (paredes) paredes->imprimir();
    if (!lecturaInicial)
        Serial.println("Sensores esperando lectura valida; comandos disponibles. El recorrido esperara datos sin terminar el intento.");
    preparado = true;
    etapaArranque = "LISTO";
    actualizarLED(); // Enciende el LED al estar listo y en la casilla inicial
    Serial.println(USAR_MAPA_SIMULADO ? "LISTO. Usar el inicio configurado del mapa." : "LISTO. Colocar en una esquina, pared detras y salida libre al frente.");
    Serial.println("- Enviar INICIAR + Enter por Serial, o");
    Serial.println("- Pulsar el boton fisico BOOT (GPIO 0).");
    Serial.println("SENSORES muestra distancias; MAPA muestra dimensiones e inicio. ! detiene y bloquea el reinicio.");
}

void loop()
{
    if (!movimiento || !navegador) {
        actualizarLED();
        delay(20); // En diagnostico no se ejecuta la navegacion.
        return;
    }
    atenderSerial();
    static unsigned long ultimoReintento = 0;
    if (!preparado && !arranqueUsado && !movimiento->fallo &&
        strcmp(etapaArranque, "ERROR_VL53L0X") == 0 && millis() - ultimoReintento >= 1000) {
        ultimoReintento = millis();
        if (fuenteParedes->begin()) {
            preparado = true;
            etapaArranque = "LISTO";
            Serial.println("LISTO. Sensores inicializados; esperando primeras lecturas.");
        }
    }
    mostrarDistanciasPeriodicamente();
    informarEspera();
    atenderBoton();
    actualizarLED();
    if (atenderCentro()) { actualizarLED(); delay(1); return; }
    if (!activo)
    {
        delay(1);
        return;
    }
    if (movimiento->fallo)
    {
        abortar();
        return;
    }
    navegador->step();
    if (!movimiento->fallo) notificarLlegadaCentro();
    if (estadoCentro != EstadoCentro::Inactivo) return;
    if (!movimiento->fallo && navegador->fase != 3)
        etapaArranque = paredes && !paredes->listo() ? "ESPERANDO_SENSORES" : "EJECUTANDO";
    actualizarLED();
    if (movimiento->fallo)
    {
        abortar();
        return;
    }
    if (navegador->fase == 3) finalizarRecorrido();
    delay(1);
}
