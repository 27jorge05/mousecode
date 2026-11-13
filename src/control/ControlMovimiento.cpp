#include "ControlMovimiento.h"

ControlMovimiento::~ControlMovimiento() {}

bool ControlMovimiento::corregir(float err, float tolOff, float tolOn, bool &activo)
{
    float a = fabsf(err);
    if (activo)
    {
        if (a <= tolOff)
            activo = false;
    }
    else
    {
        if (a >= tolOn)
            activo = true;
    }
    return activo;
}

float ControlMovimiento::limitarMag(float x, float mn, float mx)
{
    float a = fabsf(x);
    if (a < mn)
        a = mn;
    if (a > mx)
        a = mx;
    return (x >= 0.0f) ? a : -a;
}

void ControlMovimiento::iniciarAccion(const Accion &a, float s, float phi)
{
    numAcciones++;
    Serial.print(">> Accion ");
    Serial.print(numAcciones);
    Serial.print(": ");

    if (a.tipo == 'A')
    {
        float dist = a.valor * CELDA;
        s_hold += dist;
        float Dplan = (s_hold - s) - d_stop;
        if (Dplan < 0.0f)
            Dplan = 0.0f;
        s0 = s;
        s_fin = s0 + Dplan;
        perfil.iniciar(Dplan);
        modo = AVANZAR;

        Serial.print("avanzar ");
        Serial.print(a.valor);
        Serial.print(" casillas (");
        Serial.print(dist * 1000.0f);
        Serial.println(" mm)");
    }
    else if (a.tipo == 'G')
    {
        phi_hold += a.valor * DEG;
        float Ttot = phi_hold - phi;
        sg = (Ttot >= 0.0f) ? 1 : -1;
        float Tplan = fabsf(Ttot) - th_stop;
        if (Tplan < 0.0f)
            Tplan = 0.0f;
        phi0 = phi;
        phi_fin = phi0 + sg * Tplan;
        perfilGiro.iniciar(Tplan);
        modo = GIRAR;

        Serial.print("girar ");
        Serial.print(a.valor);
        Serial.println(" grados");
    }
}

void ControlMovimiento::emitirTelemetriaAngulos(Print &out, float phi_obj, float phi_enc, float phi_imu)
{
    out.print(phi_obj / DEG);
    out.print(",");
    out.print(phi_enc / DEG);
    out.print(",");
    out.println(phi_imu / DEG);
}

bool ControlMovimiento::begin()
{
    m1.detener();
    m2.detener();

    Serial.println("FAB CONTROL: iniciando encoders...");
    enc1.iniciar(100);
    enc2.iniciar(100);

    // IMU
    Serial.println("FAB CONTROL: buscando MPU6050 (SDA 25, SCL 26)...");
    imuOk = imu.begin();
    if (imuOk)
    {
        Serial.println("MPU6050 good");
        imu.calibrar(500);
        imu.reiniciar(0.0f);
    }
    else
    {
        Serial.println("MPU6050 NO detectado: el robot no se moverá");
        fallo = true;
        return false;
    }

    t_prev = micros();
    return true;
}

void ControlMovimiento::stop()
{
    m1.detener();
    m2.detener();
    pi1.reiniciar();
    pi2.reiniciar();
    modo = REPOSO;
    modo_ant = REPOSO;
    act_s = false;
    act_phi = false;
}

bool ControlMovimiento::ejecutar(char tipo, float valor)
{
    // PRUEBA SIN MOTORES: mostrar la orden, sin confirmar un movimiento inexistente.
    stop();
    Serial.print("HARIA: ");
    if (tipo == 'A')
    {
        Serial.print("avanzar ");
        Serial.print(valor);
        Serial.print(" casilla(s), ");
        Serial.print(valor * CELDA * 100.0f);
        Serial.println(" cm");
    }
    else if (tipo == 'G')
    {
        if (fabsf(valor) == 180.0f)
            Serial.print("media vuelta, ");
        else
            Serial.print(valor > 0 ? "girar izquierda, " : "girar derecha, ");
        Serial.print(fabsf(valor));
        Serial.println(" grados");
    }
    else
    {
        Serial.println("orden desconocida");
    }
    Serial.println("NO EJECUTADO. Se detiene el intento sin cambiar la posicion del algoritmo.");
    fallo = true;
    return false;

    // CONTROL REAL COMENTADO PARA ESTA PRUEBA.
    //    Conservar este bloque: contiene el movimiento original con motores.

    if (fallo || !imuOk)
        return false;

    // Medición inicial de odometría y orientación
    float s = odo.s();
    float phi = imu.getPhiFusion();

    iniciarAccion({tipo, valor}, s, phi);
    t_fin_accion = 0;
    t_asentado_ini = 0;
    t_prev = micros();

    uint32_t t_inicio_ejecutar = millis();

    while (!fallo)
    {
        // En movimiento solo se acepta parada '!': consumir cualquier texto previo para no ocultar '!'
        while (Serial.available())
        {
            if (Serial.read() == '!')
            {
                fallo = true;
                stop();
                return false;
            }
        }

        unsigned long ahora = micros();
        float dt = (ahora - t_prev) / 1000000.0f;
        t_prev = ahora;

        if (!imuOk)
        {
            pi1.calcular(0.0f, 0.0f);
            pi2.calcular(0.0f, 0.0f);
            fallo = true;
            stop();
            return false;
        }

        // Mediciones
        float velmedi1 = enc1.leerVelocidad();
        enc1.leerPosicion();
        float velmedi2 = enc2.leerVelocidad();
        enc2.leerPosicion();

        // Odometría
        odo.actualizar(enc1.getGrados(), enc2.getGrados());
        s = odo.s();
        float phi_enc = odo.phi();
        imu.actualizar(dt, phi_enc);
        phi = imu.getPhiFusion();
        float phi_imu = imu.getPhiIMU();

        float s_ref = s;
        float phi_obj = phi;
        float v_cmd = 0.0, omega_cmd = 0.0;

        if (modo == AVANZAR)
        {
            perfil.actualizar(dt);
            s_ref = s0 + perfil.s();
            phi_obj = phi_hold;

            if (perfil.terminado() && s >= s_fin)
            {
                modo = REPOSO;
            }
            else
            {
                float v_ff = perfil.v() + TAU_FF * perfil.a();
                v_cmd = lazoS.calcular(s_ref, s, v_ff);
                if (v_cmd < perfil.getVmin())
                    v_cmd = perfil.getVmin();

                omega_cmd = lazoPhi.calcular(phi_hold, phi);
                float lim = fminf(OMEGA_MAX_AVAN, 2.0f * v_cmd / BASE);
                omega_cmd = constrain(omega_cmd, -lim, lim);
            }
        }
        else if (modo == GIRAR)
        {
            perfilGiro.actualizar(dt);
            phi_obj = phi0 + sg * perfilGiro.s();

            if (perfilGiro.terminado() && sg * (phi - phi_fin) >= 0.0f)
            {
                modo = REPOSO;
            }
            else
            {
                float w_ff = sg * (perfilGiro.v() + TAU_FF * perfilGiro.a());
                omega_cmd = lazoPhi.calcular(phi_obj, phi, w_ff);
                if (sg * omega_cmd < perfilGiro.getVmin())
                    omega_cmd = sg * perfilGiro.getVmin();
            }
        }

        if (modo == REPOSO)
        {
            s_ref = s_hold;
            phi_obj = phi_hold;
            float e_s = s_hold - s;
            float e_phi = phi_hold - phi;
            if (corregir(e_s, TOL_S, TOL_S_ON, act_s))
            {
                v_cmd = limitarMag(lazoS.getKp() * e_s, V_HOLD_MIN, V_HOLD_MAX);
            }
            if (corregir(e_phi, TOL_PHI, TOL_PHI_ON, act_phi))
            {
                omega_cmd = limitarMag(lazoPhi.getKp() * e_phi, W_HOLD_MIN, W_HOLD_MAX);
            }
        }

        if (modo_ant != REPOSO && modo == REPOSO)
        {
            t_fin_accion = millis();
            t_asentado_ini = 0;
        }
        modo_ant = modo;

        // Criterio para cambiar de referencia: error pequeño y robot quieto
        if (modo == REPOSO)
        {
            if (t_fin_accion == 0)
                t_fin_accion = millis();

            float e_s = s_hold - s;
            float e_phi = phi_hold - phi;
            bool dentro = fabsf(e_s) <= TOL_S_FIN &&
                          fabsf(e_phi) <= TOL_PHI_FIN &&
                          !act_s && !act_phi &&
                          fabsf(velmedi1) < VEL_ASENTADO_RPM &&
                          fabsf(velmedi2) < VEL_ASENTADO_RPM;

            if (dentro)
            {
                if (t_asentado_ini == 0)
                    t_asentado_ini = millis();
            }
            else
            {
                t_asentado_ini = 0;
            }

            unsigned long en_reposo = millis() - t_fin_accion;
            bool asentado = (t_asentado_ini != 0) &&
                            (millis() - t_asentado_ini >= T_ASENTADO_MS) &&
                            (en_reposo >= T_ESPERA_MIN_MS);
            bool agotado = (en_reposo >= T_ASENTAR_MAX_MS);

            if (asentado || agotado)
            {
                Serial.print(asentado ? "   asentado: " : "   AVISO, no asento: ");
                Serial.print("e_dist = ");
                Serial.print(e_s * 1000.0f);
                Serial.print(" mm, e_ang = ");
                Serial.print(e_phi / DEG);
                Serial.print(" grados, t = ");
                Serial.print(en_reposo);
                Serial.println(" ms");

                stop();
                return true;
            }
        }

        float wL_ref, wR_ref;
        mezcla.mezclar(v_cmd, omega_cmd, wL_ref, wR_ref);

        pi1.calcular(wL_ref, velmedi1);
        pi2.calcular(wR_ref, velmedi2);

#if TELEMETRIA
        static uint8_t n = 0;
        if (++n >= 5)
        {
            n = 0;
            // Header: phi_obj_deg, phi_enc_deg, phi_imu_deg
            emitirTelemetriaAngulos(Serial, phi_obj, phi, phi_imu);
        }
#endif

        if (millis() - t_inicio_ejecutar > 30000)
        {
            Serial.println("ERROR: timeout de movimiento");
            fallo = true;
            break;
        }

        delay(10);
    }

    stop();
    return false;
}
