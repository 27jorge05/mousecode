#include "SensoresPared.h"

FuenteParedes::~FuenteParedes() {}

bool SensoresParedHardcodeados::begin() const  {
        using namespace MapaSimulador;
        if (FILAS < 2 || COLUMNAS < 2 || FILAS * COLUMNAS > 256 ||
            INICIO_FILA < 0 || INICIO_FILA >= FILAS || INICIO_COLUMNA < 0 || INICIO_COLUMNA >= COLUMNAS ||
            INICIO_RUMBO < 0 || INICIO_RUMBO > 3) return false;
        for (int r = 0; r < FILAS; ++r) for (int c = 0; c < COLUMNAS; ++c) {
            const uint8_t m = PAREDES[r * COLUMNAS + c];
            if (m > 15 || (r == 0 && !(m & 1)) || (c == COLUMNAS-1 && !(m & 2)) ||
                (r == FILAS-1 && !(m & 4)) || (c == 0 && !(m & 8))) return false;
            if (c+1 < COLUMNAS && bool(m & 2) != bool(PAREDES[r * COLUMNAS + c+1] & 8)) return false;
            if (r+1 < FILAS && bool(m & 4) != bool(PAREDES[(r+1) * COLUMNAS + c] & 1)) return false;
        }
        const uint8_t inicio=PAREDES[INICIO_FILA * COLUMNAS + INICIO_COLUMNA];
        int salidas=0;
        for(int d=0;d<4;++d)if(!(inicio&(1<<d)))++salidas;
        const bool esquina = (INICIO_FILA == 0 || INICIO_FILA == FILAS - 1) &&
                             (INICIO_COLUMNA == 0 || INICIO_COLUMNA == COLUMNAS - 1);
        return esquina && (salidas == 1 || salidas == 2) &&
               !(inicio & (1 << INICIO_RUMBO)) && (inicio & (1 << ((INICIO_RUMBO + 2) % 4)));
    }

bool SensoresParedHardcodeados::leer(int lado, int fila, int columna, int rumbo) const  {
        if (lado < 0 || lado > 2 || fila < 0 || columna < 0 || fila >= MapaSimulador::FILAS ||
            columna >= MapaSimulador::COLUMNAS || rumbo < 0 || rumbo > 3) return true;
        const int relativo = lado == 1 ? 3 : (lado == 2 ? 1 : 0);
        const int direccion = (rumbo + relativo) % 4;
        return MapaSimulador::PAREDES[fila * MapaSimulador::COLUMNAS + columna] & (1 << direccion);
    }
