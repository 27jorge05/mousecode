#pragma once
#include "../mapa/MapaSimulador.h"

namespace ConfiguracionUnificada {
// Un ciclo completo es inicio -> centro -> inicio. El medio final termina en el centro.
constexpr float CICLOS = 2.5f;
constexpr int LLEGADAS = static_cast<int>(CICLOS + 0.5f);
constexpr int ZONAS_POR_RETORNO = 8;
static_assert(CICLOS >= 2.5f && CICLOS <= 15.5f && LLEGADAS == CICLOS + 0.5f,
              "Ciclos validos: 2.5, 3.5, ..., 15.5");
static_assert(ZONAS_POR_RETORNO >= 0 && ZONAS_POR_RETORNO <= 32, "Usar de 0 a 32 zonas");

// Mismas consignas máximas y geometría que el control recibido. Unidades SI.
constexpr float CELDA_M = MapaSimulador::CASILLA_CM / 100.0f;
constexpr float VELOCIDAD_M_S = 0.50f;
constexpr float ACELERACION_M_S2 = 1.00f;
constexpr float VELOCIDAD_MIN_M_S = 0.10f;
constexpr float GIRO_GRADOS_S = 400.0f;
constexpr float ACELERACION_GIRO_GRADOS_S2 = 1000.0f;
} // namespace ConfiguracionUnificada
