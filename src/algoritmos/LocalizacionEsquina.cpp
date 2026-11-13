#include "LocalizacionEsquina.h"

LocalizacionEsquina::Estado LocalizacionEsquina::fallar(const char* motivo) {
    motivoError = motivo;
    avancePermitido = false;
    return estadoActual = Estado::Error;
}

bool LocalizacionEsquina::iniciar(int filas, int columnas) {
    avances = 0;
    avancePermitido = false;
    ladoInterior = Interior::Desconocido;
    motivoError = "";
    // Sin saber la esquina fisica no conocemos cual eje es el largo en un rectangulo.
    if (filas != columnas || filas < 2 || filas > MAX_LADO) {
        fallar("La localizacion requiere un laberinto cuadrado de 2 a 16 casillas por lado.");
        return false;
    }
    lado = filas;
    estadoActual = Estado::BuscandoInterior;
    return true;
}

LocalizacionEsquina::Estado LocalizacionEsquina::observar(
    bool frente, bool izquierda, bool derecha) {
    if (estadoActual != Estado::BuscandoInterior) return estadoActual;
    avancePermitido = false;
    if (avances == 0 && frente)
        return fallar("Inicio invalido: colocar en una esquina apuntando a una salida libre.");
    if (!izquierda && !derecha)
        return fallar("Ambos laterales abiertos antes del primer giro: revisar posicion y lecturas.");
    historial[avances] = {frente, izquierda, derecha};
    if (izquierda != derecha) {
        ladoInterior = izquierda ? Interior::Derecha : Interior::Izquierda;
        return estadoActual = Estado::Localizado;
    }
    if (frente)
        return fallar("Pasillo inicial cerrado sin abertura lateral: no se identifico el interior.");
    if (avances >= lado - 1)
        return fallar("El recorrido inicial excederia las dimensiones del laberinto.");
    avancePermitido = true;
    return estadoActual;
}

bool LocalizacionEsquina::confirmarAvance() {
    if (estadoActual != Estado::BuscandoInterior || !avancePermitido) return false;
    ++avances;
    avancePermitido = false;
    return true;
}
