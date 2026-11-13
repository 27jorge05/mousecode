// UNIFICACION: fuente de paredes intercambiable y localizacion inicial opcional.
#pragma once
#include "../control/ControlMovimiento.h"
#include "../sensores/SensoresPared.h"
#include "LocalizacionEsquina.h"
class CiclicoRecto {
public:
explicit CiclicoRecto(ControlMovimiento &r, FuenteParedes &p, bool localizarInicio = false);
FuenteParedes &paredes;
bool localizacionAutomatica = false;
bool localizando = false;
int patronLocalizacion = -1;
int capturasLocalizacion = 0;
int filaOrigen = 15, columnaOrigen = 0;
LocalizacionEsquina localizacion;
bool centroLocalizado() const;
void pasoLocalizacion();
void finalizarLocalizacion();
void fallarLocalizacion(const char* motivo);
// Cíclico: explorar → volver aprendiendo → carrera por rectas largas
// Estrategia propia; conserva mapa y mejor ruta entre carreras.
// A* temporal con preferencia por rectas; carrera con pivotes, sin suavizado.
// Fuente: https://micromouse.at/en/software.html
// A* online: mapa optimista y heurística Manhattan admisible.
// MouseLab — IMPLEMENTACIÓN PROPIA a partir de A*, Dijkstra y Flood Fill.
// Referencia: artículo de Mishra y Bande, enlazado abajo.
// https://swati-mishra.com/wp-content/uploads/2020/02/advanced-algorithms-for-micro-mouse-maze-solving-las-vegas.pdf
// Solo sensores, odometría y configuración pública; no lee el mapa del simulador.
// Arreglos fijos para hasta 256 celdas. Arreglos: aproximadamente 23 KB de SRAM; usar ESP32 o placa con margen suficiente.
// Fases: 0 explorar, 1 volver al inicio, 2 carrera sobre mapa aprendido, 3 terminar.

static constexpr int CAPACIDAD = 256;
const int TIPO = 1;
int carrerasCompletadas = 0;
int maxCarreras = 3;
int maxDesvios = 8;
int desviosRetorno = 0;
int desviosTotales = 0;
int objetivoExploracion = -1;
int longitudMejor = 0;
float tiempoMejor = 30000;
float penalizacionGiro = 0.6;
uint16_t mejorRuta[1024];
uint16_t distMapa[256];
uint16_t distActual[256];
uint16_t distInicioOpt[256];
uint16_t distMetaOpt[256];
uint16_t distInicioConfirmado[256];
bool trayectoriaPreparada = false;
const int INFINITO = 30000;
int filas = 16;
int columnas = 16;
int cantidad = 256;
int fila = 15;
int columna = 0;
int direccion = 0;
int inicio = 240;
int fase = 0;
int observadas = 0;
int planificaciones = 0;
int movimientosAlgoritmo = 0;
int nivel = 0;
int longitudRuta = 0;
int indiceRuta = -1;
bool centro = true;
bool mapaCambio = true;
bool destinoInicio = false;
bool reinicioFueraDelInicio = false;
bool soloConocidos = false;
uint8_t muros[256];
uint8_t conocidos[256];
bool visitadas[256];
uint8_t trabajo[256];
float distancias[1024];
uint16_t padres[1024];
bool cerrados[1024];
uint16_t heap[1024];
int16_t posiciones[1024];
float claves[1024];
int abiertosCantidad = 0;
float costoPlan = 0;
int origenPlan = 0;
int rumboPlan = 0;
uint16_t ruta[1024];

int celdaActual();

int vecino(int celda, int dir);

bool esMeta(int celda);

bool esDestino(int celda);

int heuristica(int celda);

bool pasoDisponible(int celda, int dir);

void registrar(int dir, bool pared);

bool actualizarSensores();
bool observar();

void moverHacia(int siguiente);

void inicializarMapa(bool leerAhora = true);

void sincronizarRecolocacion();

// Selección lineal del mínimo en el conjunto abierto, sin memoria dinámica.
// A*: f=g+h. Dijkstra: h=0. Coste de cada arista: una casilla.
float costoHeuristica(int estado);
float costoArista(int cuartos, int celdas);
// Cola binaria con decrease-key y capacidad fija, una entrada por estado.
void ponerEnHeap(int estado);
int sacarHeap();

void planificarBusqueda();

// BFS para priorizar fronteras que pueden mejorar el camino inicio-centro.
// El modo confirmado solo usa pasos leídos; el optimista admite lo desconocido.
void llenarDistancias(int modo);
void elegirFrontera();
void prepararCandidata();
void prepararMejorGuardada();
void elegirCarreraRapida();
void comenzarRetorno();
void seguirBusqueda();
void actualizarFases();
void begin();
void step();


ControlMovimiento &robot;
int siguienteTramo = 1;
int trayectoria[1025] = {}; int puntos = 0; bool terminada = true;
uint8_t rutaMuros[256] = {}, rutaConocidos[256] = {};
int filasLaberinto(); int columnasLaberinto();
int filaInicial(); int columnaInicial();
int filaActual(); int columnaActual();
int direccionActual(); bool metaCentral();
bool enInicio();
int limiteCarreras(); int presupuestoExploracion();
float pasoCentimetros();
float velocidadMaxima();
float duracion(float d, float vmax, float acc, float vmin);
float tiempoRecta(int n);
float tiempoGiro(int q);
bool paredFrente();
bool paredIzquierda();
bool paredDerecha();
void girarDerecha();
void girarIzquierda();
void mediaVuelta();
void avanzar();
void detener();
void prepararRuta(int);
void mapaRuta(int c,int m,int k);
void puntoRuta(int c);
bool rutaTerminada();
int rumboEntre(int a,int b);
float tiempoRutaEstimada();
void seguirRuta();

};
