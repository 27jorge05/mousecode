#pragma once
#include <stdint.h>

// DETECCION HARDCODEADA: mapa del simulador, NO sensores IR.
// Editable como vector. Indice = fila * COLUMNAS + columna.
// Bits de pared: norte=1, este=2, sur=4, oeste=8. 0=abierto, 15=cerrado.
// Regenerar con: python3 exportar_mapa.py micromouse-centro.json
namespace MapaSimulador {
constexpr int FILAS = 16;
constexpr int COLUMNAS = 16;
constexpr int INICIO_FILA = 15;
constexpr int INICIO_COLUMNA = 0;
constexpr int INICIO_RUMBO = 0; // norte=0, este=1, sur=2, oeste=3
constexpr float CASILLA_CM = 18.000000f;
constexpr char ARCHIVO[] = "micromouse-centro.json";
constexpr char SHA256[] = "f50fdafd49a2d4230351bf946b7ccdaf3936b4f5840ea640e2b164784063c77c";
constexpr uint8_t PAREDES[] = {
     9,  5,  1,  1,  3,  9,  5,  1,  5,  1,  5,  1,  5,  1,  3, 11, // fila 0
     8,  1,  2, 10,  8,  2,  9,  2, 11,  8,  1,  4,  1,  6, 12,  2, // fila 1
    14, 10,  8,  4,  2, 12,  6,  8,  2, 10,  8,  3,  8,  3,  9,  6, // fila 2
     9,  4,  0,  1,  0,  1,  5,  2, 10,  8,  2,  8,  2, 10,  8,  7, // fila 3
    10,  9,  2, 14, 10,  8,  1,  0,  4,  2,  8,  2,  8,  4,  0,  3, // fila 4
    10, 10, 12,  1,  4,  6, 10,  8,  5,  0,  2,  8,  6,  9,  6, 10, // fila 5
     8,  0,  5,  4,  3,  9,  4,  6, 13,  2, 12,  4,  5,  2,  9,  2, // fila 6
    10, 12,  1,  3,  8,  4,  1,  1,  3,  8,  1,  1,  3,  8,  6, 10, // fila 7
    10, 13,  2,  8,  4,  1,  2, 12,  6, 10, 10, 10,  8,  4,  3, 10, // fila 8
     8,  5,  2, 10,  9,  6,  8,  1,  1,  0,  4,  2, 10, 11,  8,  2, // fila 9
    10,  9,  4,  2, 12,  5,  2, 10, 10, 12,  7, 10, 10,  8,  2, 10, // fila 10
    10,  8,  5,  4,  1,  5,  2, 12,  0,  5,  5,  0,  4,  2, 12,  2, // fila 11
     8,  0,  5,  5,  2, 13,  6,  9,  2,  9,  1,  4,  3,  8,  3, 10, // fila 12
    14,  8,  1,  3, 12,  3,  9,  2, 10, 10,  8,  5,  2, 10, 12,  2, // fila 13
     9,  2, 10, 12,  1,  4,  2, 10,  8,  4,  0,  3, 10, 12,  1,  2, // fila 14
    14, 12,  4,  5,  4,  5,  4,  4,  4,  5,  6, 12,  4,  5,  6, 14, // fila 15
};
static_assert(sizeof(PAREDES) / sizeof(PAREDES[0]) == FILAS * COLUMNAS, "Revisar longitud del vector de paredes");
static_assert(FILAS * COLUMNAS <= 256, "Capacidad maxima: 256 casillas");
} // namespace MapaSimulador
