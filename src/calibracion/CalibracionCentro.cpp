#include "CalibracionCentro.h"
#include <math.h>
using namespace ConfiguracionCalibracionCentro;

void CalibracionCentro::AjusteRecta::agregar(float distanciaCm, float paredCm) {
    if (n < 5 && isfinite(distanciaCm) && isfinite(paredCm)) {
        x[n] = distanciaCm; y[n] = paredCm; ++n;
    }
}

bool CalibracionCentro::AjusteRecta::estimar(float& pendiente, float& residuoCm) const {
    if (n < 3 || x[n-1] - x[0] < 9.0f) return false;
    float sx=0, sy=0, sxx=0, sxy=0;
    for (int i=0;i<n;++i) { sx+=x[i]; sy+=y[i]; sxx+=x[i]*x[i]; sxy+=x[i]*y[i]; }
    const float denominador=n*sxx-sx*sx;
    if (denominador < 0.001f) return false;
    pendiente=(n*sxy-sx*sy)/denominador;
    const float intercepto=(sy-pendiente*sx)/n;
    residuoCm=0;
    for (int i=0;i<n;++i) residuoCm=fmaxf(residuoCm,fabsf(y[i]-intercepto-pendiente*x[i]));
    return isfinite(pendiente) && residuoCm <= RESIDUO_MAX_CM;
}

float CalibracionCentro::errorAngularGrados(float pendiente, bool izquierda) {
    // Con sensores perpendiculares: pendiente izquierda=-tan(theta), derecha=tan(theta).
    return (izquierda ? -1.0f : 1.0f) * atanf(pendiente) * 180.0f / PI;
}

void CalibracionCentro::fallar(const char* razon) {
    motivo=razon; actual=Estado::Error; control.stop();
    Serial.print("ERROR CALIBRACION CENTRO: "); Serial.println(razon);
}

void CalibracionCentro::cancelar() { fallar("Cancelada; no se reanuda con una posicion sin verificar."); }

void CalibracionCentro::esperar(Estado siguiente) {
    sensores.reiniciarFiltro(); inicioEspera=millis(); actual=siguiente;
}

bool CalibracionCentro::iniciar(int fila, int columna, int direccion, int filas, int columnas) {
    control.stop(); motivo=""; inicioRutina=millis();
    if (control.fallo || filas<2 || columnas<2 || filas%2 || columnas%2 || direccion<0 || direccion>3) {
        fallar("Se requiere centro 2x2 y control disponible."); return false;
    }
    const int r=filas/2-1, c=columnas/2-1;
    if (fila<r || fila>r+1 || columna<c || columna>c+1) {
        fallar("El robot no esta en una celda central."); return false;
    }
    // Orden horario: superior izquierda, superior derecha, inferior derecha, inferior izquierda.
    origen = fila==r ? (columna==c ? 0 : 1) : (columna==c ? 3 : 2);
    celda=origen; rumbo=rumboInicial=direccion; vuelta=tramos=paso=0;
    referencias[0]=referencias[1]=0;
    for(int i=0;i<3;++i) referenciaInicialValida[i]=false;
    esperar(Estado::ReferenciaInicial);
    Serial.println("CALIBRACION CENTRO: dos vueltas opuestas; referencia izquierda y derecha. Usar zona central despejada.");
    return true;
}

bool CalibracionCentro::capturar() {
    sensores.actualizar();
    if (sensores.lecturaVigente(3)) return true;
    if (millis()-inicioEspera > ESPERA_LECTURA_MS) fallar("Sin lectura frontal reciente.");
    return false;
}

bool CalibracionCentro::ejecutar(char tipo, float valor) {
    if (!control.ejecutar(tipo,valor) || control.fallo) {
        fallar("Movimiento no completado; se conserva detenido."); return false;
    }
    return true;
}

bool CalibracionCentro::orientar(int destino) {
    const int q=(destino-rumbo+4)%4;
    if(q && !ejecutar('G',q==1 ? -90.0f : q==3 ? 90.0f : 180.0f)) return false;
    rumbo=destino; return true;
}

void CalibracionCentro::actualizar() {
    if (actual==Estado::Inactiva || actual==Estado::Terminada || actual==Estado::Error) return;
    if (millis()-inicioRutina>DURACION_MAX_MS) { fallar("Tiempo total de calibracion agotado."); return; }
    if (control.fallo && actual!=Estado::Error) { fallar("Fallo del control."); return; }
    switch(actual) {
    case Estado::ReferenciaInicial:
        if(!capturar()) return;
        for(int i=0;i<3;++i) {
            const int id= i==0 ? 1 : i==1 ? 3 : 5;
            referenciaInicialValida[i]=sensores.lecturaVigente(id);
            distanciasIniciales[i]=sensores.lectura(id).filtradaCm;
        }
        actual=Estado::Orientar; return;
    case Estado::Orientar: {
        // Horario: E,S,O,N. Antihorario: S,O,N,E desde las mismas cuatro celdas.
        const int horario[4]={1,2,3,0};
        const int antihorario[4]={2,3,0,1};
        if(!orientar(vuelta==0 ? horario[celda] : antihorario[celda])) return;
        ajustes[0]=AjusteRecta{}; ajustes[1]=AjusteRecta{};
        paso=0; inicioDistancia=control.odo.s();
        esperar(Estado::Medir); return;
    }
    case Estado::Medir: {
        if(!capturar()) return;
        const int ids[2]={1,5};
        const float distancia=(control.odo.s()-inicioDistancia)*100.0f;
        for(int lado=0;lado<2;++lado) {
            if(!sensores.lecturaVigente(ids[lado])) continue;
            const float d=sensores.lectura(ids[lado]).filtradaCm;
            const int indice=lado==0 ? 0 : 2;
            if(d<=ConfiguracionVL53L0X::UMBRALES_CM[indice]) {
                if(d<MARGEN_LATERAL_CM) { fallar("Demasiado cerca de una pared lateral."); return; }
                ajustes[lado].agregar(distancia,d);
            }
        }
        if(paso==PASOS_POR_CELDA) { actual=Estado::Corregir; return; }
        const float pasoCm=control.CELDA*100.0f/PASOS_POR_CELDA;
        if(sensores.lectura(3).filtradaCm<=pasoCm+MARGEN_FRONTAL_CM) {
            fallar("Sin espacio frontal para el siguiente paso."); return;
        }
        distanciaAntesPaso=control.odo.s(); actual=Estado::Avanzar; return;
    }
    case Estado::Avanzar: {
        if(!ejecutar('A',1.0f/PASOS_POR_CELDA)) return;
        const float medido=control.odo.s()-distanciaAntesPaso;
        const float previsto=control.CELDA/PASOS_POR_CELDA;
        if(medido<previsto*0.5f || medido>previsto*1.5f) { fallar("Avance de encoders incompatible con el paso solicitado."); return; }
        ++paso; esperar(Estado::Medir); return;
    }
    case Estado::Corregir: {
        float angulos[2]={}, pendientes[2]={}, residuos[2]={}; bool validos[2]={};
        for(int lado=0;lado<2;++lado) {
            validos[lado]=ajustes[lado].estimar(pendientes[lado],residuos[lado]);
            if(validos[lado]) {
                angulos[lado]=errorAngularGrados(pendientes[lado],lado==0);
                Serial.print(lado==0 ? "Referencia IZQ: error angular=" : "Referencia DER: error angular=");
                Serial.println(angulos[lado]);
            }
        }
        // La pared exterior es izquierda en horario y derecha en antihorario.
        const int exterior=vuelta;
        if(validos[exterior]) {
            if(validos[1-exterior] && fabsf(angulos[0]-angulos[1])>DESACUERDO_MAX_GRADOS) {
                fallar("Las referencias laterales no concuerdan; no se aplica correccion."); return;
            }
            const float angulo=validos[0]&&validos[1] ? (angulos[0]+angulos[1])*0.5f : angulos[exterior];
            if(fabsf(angulo)>CORRECCION_MAX_GRADOS) { fallar("Desalineacion excesiva; revisar montaje y control."); return; }
            if(fabsf(angulo)>=CORRECCION_MIN_GRADOS && !ejecutar('G',-angulo)) return;
            ++referencias[exterior];
        } else Serial.println("Tramo sin referencia exterior continua: no se aplica correccion.");
        celda=(celda+(vuelta==0 ? 1 : 3))%4;
        if(++tramos==4) {
            tramos=0;
            if(++vuelta==2) { actual=Estado::RestaurarRumbo; return; }
        }
        actual=Estado::Orientar; return;
    }
    case Estado::RestaurarRumbo:
        if(!orientar(rumboInicial)) return;
        if(celda!=origen || !referencias[0] || !referencias[1]) { fallar("Faltaron referencias de pared en uno de los dos lados."); return; }
        esperar(Estado::VerificarRetorno); return;
    case Estado::VerificarRetorno:
        if(!capturar()) return;
        for(int i=0;i<3;++i) {
            const int id=i==0 ? 1 : i==1 ? 3 : 5;
            if(!referenciaInicialValida[i]) continue;
            if(!sensores.lecturaVigente(id)) {
                if(millis()-inicioEspera>ESPERA_LECTURA_MS) fallar("Falta una referencia para verificar el retorno.");
                return;
            }
            if(fabsf(sensores.lectura(id).filtradaCm-distanciasIniciales[i])>TOL_RETORNO_CM) {
                fallar("La pose final no coincide con las referencias iniciales."); return;
            }
        }
        control.stop(); sensores.reiniciarFiltro(); actual=Estado::Terminada;
        Serial.println("CALIBRACION CENTRO TERMINADA: referencias laterales verificadas. No se modificaron polinomios ni escala del giroscopio.");
        return;
    default: return;
    }
}
