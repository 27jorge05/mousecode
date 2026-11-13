#pragma once
#include <Arduino.h>
#include <ESP32Encoder.h>
#include "../configuracion/ConfiguracionUnificada.h"
#include "../motor/motor.h"
#include "../controlpi/controlpi.h"
#include "../codificador/codificador.h"
#include "../odometria/odometria.h"
#include "../lazop/lazop.h"
#include "../mezclador/mezclador.h"
#include "../perfil/perfil.h"
#include "../giroscopio/fusionangular.h"

class ControlMovimiento {
public:
    virtual ~ControlMovimiento();
    bool fallo = false;

    // PI velocidad
    const float Ki1 = 1.4697, Kp1 = 0.1396;
    const float Ki2 = 1.4276, Kp2 = 0.1330;
    const float Ts  = 0.01;

    // Motor 1 IZQ (marcado)
    #define M1_IN1   5
    #define M1_IN2   18
    #define M1_EN    22
    #define M1_ENC   4   // verde
    #define M1_ENC_B 34  // amarillo
    // Motor 2 DER
    #define M2_IN1   19
    #define M2_IN2   21
    #define M2_EN    23
    #define M2_ENC   35  // amarillo
    #define M2_ENC_B 13  // verde
    const float PPR = 200.0;

    // IMU I2C
    #define IMU_SDA  25
    #define IMU_SCL  26
    // Direccion identificada en la prueba del MPU conectado por el usuario.
    static constexpr uint8_t IMU_DIRECCION = 0x69;

    // Parámetros del robot
    const float R_RUEDA  = 0.02205;
    // const float BASE = 0.0722; 
    const float BASE = 0.0694;
    const float RPM_MAX  = 250.0;
    const float RPM_A_MS = 2.0 * PI * R_RUEDA / 60.0;
    const float DEG      = PI / 180.0;

    // Parámetros de control
    const float TAU_FF         = 0.15;
    const float OMEGA_MAX_AVAN = 2.5;
    const float W_TOPE         = 2.0 * RPM_MAX * RPM_A_MS / BASE;

    const float V_MAX = 0.50;
    const float A_MAX = 1.00;
    const float V_MIN = 0.10;

    const float W_MAX    = 400.0 * DEG;
    const float ALFA_MAX = 1000.0 * DEG;
    const float W_MIN    = 2.0 * V_MIN / BASE;

    const float TOL_S      = 0.0005;     // [m]   
    const float TOL_S_ON   = 0.0015;     // [m]  
    const float TOL_PHI    = 0.3 * DEG;  // [rad]
    const float TOL_PHI_ON = 1.0 * DEG;  // [rad]
    const float V_HOLD_MIN = 0.03;
    const float V_HOLD_MAX = 0.20;
    const float W_HOLD_MIN = 0.8;
    const float W_HOLD_MAX = 2.0;

    // Criterio para cambiar de referencia, error pequeño y robot quieto
    const float TOL_S_FIN   = TOL_S_ON;          // [m]   
    const float TOL_PHI_FIN = TOL_PHI_ON;        // [rad] 
    const float VEL_ASENTADO_RPM = 10.0;         // 
    const unsigned long T_ESPERA_MIN_MS   = 200; // 
    const unsigned long T_ASENTADO_MS     = 150; // 
    const unsigned long T_ASENTAR_MAX_MS  = 3000;// 

    // Telemetría
    #define TELEMETRIA  0

    // Objetos
    motor       m1{M1_IN1, M1_IN2, M1_EN};
    motor       m2{M2_IN1, M2_IN2, M2_EN};
    ControlPI   pi1{m1, Kp1, Ki1, Ts, 0.0, 100.0};
    ControlPI   pi2{m2, Kp2, Ki2, Ts, 0.0, 100.0};
    Codificador enc1{M1_ENC, PPR, M1_ENC_B};
    Codificador enc2{M2_ENC, PPR, M2_ENC_B};

    Odometria     odo{R_RUEDA, BASE};
    LazoP         lazoS{1.67f, 0.0f, RPM_MAX * RPM_A_MS};
    LazoP         lazoPhi{1.67f, -W_TOPE, W_TOPE};
    Perfil        perfil{V_MAX, A_MAX, V_MIN};
    Perfil        perfilGiro{W_MAX, ALFA_MAX, W_MIN};
    Mezclador     mezcla{BASE, R_RUEDA, RPM_MAX};
    FusionAngular imu{IMU_SDA, IMU_SCL, IMU_DIRECCION};

    // Máquina de estados
    enum Modo { REPOSO, AVANZAR, GIRAR };
    Modo modo     = REPOSO;
    Modo modo_ant = REPOSO;

    float s0       = 0.0;
    float s_fin    = 0.0;
    float s_hold   = 0.0;
    float phi_hold = 0.0;
    bool  act_s = false, act_phi = false;
    float d_stop   = 0.045;

    float  phi0    = 0.0;
    float  phi_fin = 0.0;
    int8_t sg      = 1;
    float  th_stop = 20.0 * DEG;

    unsigned long t_prev = 0;
    bool imuOk = false;

    const float CELDA = 0.18;   // [m] distancia por casilla

    struct Accion {
        char  tipo;   // 'A' avanzar, 'G' girar
        float valor;  // casillas (A) o grados (G, + izquierda, - derecha)
    };

    int           numAcciones    = 0;
    unsigned long t_fin_accion   = 0;   
    unsigned long t_asentado_ini = 0;   

    bool corregir(float err, float tolOff, float tolOn, bool &activo);
    float limitarMag(float x, float mn, float mx);
    void iniciarAccion(const Accion &a, float s, float phi);
    void emitirTelemetriaAngulos(Print &out, float phi_obj, float phi_enc, float phi_imu);

    bool begin();
    virtual void stop();
    virtual bool ejecutar(char tipo, float valor);
};
