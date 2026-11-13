#include "CiclicoCurvo.h"

CiclicoCurvo::CiclicoCurvo(ControlMovimiento &r, FuenteParedes &p, bool localizarInicio)
    : paredes(p), localizacionAutomatica(localizarInicio), robot(r) {}

int CiclicoCurvo::celdaActual() {
  return fila * columnas + columna;
}

int CiclicoCurvo::vecino(int celda, int dir) {
  int r = celda / columnas;
  int c = celda % columnas;
  if (dir == 0) { if (r == 0) return -1; return celda - columnas; }
  if (dir == 1) { if (c == columnas - 1) return -1; return celda + 1; }
  if (dir == 2) { if (r == filas - 1) return -1; return celda + columnas; }
  if (c == 0) return -1;
  return celda - 1;
}

bool CiclicoCurvo::esMeta(int celda) {
  if (!centro) return celda == columnas - 1;
  int r = celda / columnas;
  int c = celda % columnas;
  return r >= (filas - 1) / 2 && r <= filas / 2 && c >= (columnas - 1) / 2 && c <= columnas / 2;
}

bool CiclicoCurvo::esDestino(int celda) {
  if (objetivoExploracion >= 0) return celda == objetivoExploracion;
  if (destinoInicio) return celda == inicio;
  return esMeta(celda);
}

int CiclicoCurvo::heuristica(int celda) {
  int r = celda / columnas;
  int c = celda % columnas;
  int r0 = (filas - 1) / 2;
  int r1 = filas / 2;
  int c0 = (columnas - 1) / 2;
  int c1 = columnas / 2;
  if (destinoInicio) { r0 = inicio / columnas; r1 = r0; c0 = inicio % columnas; c1 = c0; }
  else if (!centro) { r0 = 0; r1 = 0; c0 = columnas - 1; c1 = c0; }
  if (objetivoExploracion >= 0) { r0 = objetivoExploracion / columnas; r1 = r0; c0 = objetivoExploracion % columnas; c1 = c0; }
  int h = 0;
  if (r < r0) h = h + r0 - r;
  if (r > r1) h = h + r - r1;
  if (c < c0) h = h + c0 - c;
  if (c > c1) h = h + c - c1;
  return h;
}

bool CiclicoCurvo::pasoDisponible(int celda, int dir) {
  int siguiente = vecino(celda, dir);
  if (siguiente < 0) return false;
  int bit = 1 << dir;
  int opuesto = 1 << ((dir + 2) % 4);
  if ((muros[celda] & bit) != 0 || (muros[siguiente] & opuesto) != 0) return false;
  if (soloConocidos && ((conocidos[celda] & bit) == 0 || (conocidos[siguiente] & opuesto) == 0)) return false;
  return true;
}

void CiclicoCurvo::registrar(int dir, bool pared) {
  int actual = celdaActual();
  int bit = 1 << dir;
  int siguiente = vecino(actual, dir);
  if (soloConocidos && (conocidos[actual] & bit) == 0) mapaCambio = true;
  conocidos[actual] = conocidos[actual] | bit;
  if (pared) {
    if ((muros[actual] & bit) == 0) mapaCambio = true;
    muros[actual] = muros[actual] | bit;
  }
  if (siguiente >= 0) {
    int opuesto = 1 << ((dir + 2) % 4);
    conocidos[siguiente] = conocidos[siguiente] | opuesto;
    if (pared) muros[siguiente] = muros[siguiente] | opuesto;
  }
}

bool CiclicoCurvo::actualizarSensores() {
  if (paredes.actualizar()) return true;
  robot.stop(); // Esperar otra lectura sin terminar el intento ni inventar paredes.
  return false;
}

bool CiclicoCurvo::observar() {
  if (!actualizarSensores()) return false;
  int actual = celdaActual();
  if (!visitadas[actual]) { visitadas[actual] = true; observadas++; }
  registrar(direccion, paredFrente());
  registrar((direccion + 3) % 4, paredIzquierda());
  registrar((direccion + 1) % 4, paredDerecha());
  // La pared trasera ya se conoce por el paso anterior o por el borde del inicio.
  return true;
}

void CiclicoCurvo::moverHacia(int siguiente) {
  int actual = celdaActual();
  int nueva = -1;
  for (int dir = 0; dir < 4; dir++) {
    if (vecino(actual, dir) == siguiente) nueva = dir;
  }
  if (nueva < 0 || !pasoDisponible(actual, nueva)) { fase = 3; detener(); return; }
  int giro = (nueva - direccion + 4) % 4;
  if (giro == 1) girarDerecha();
  if (giro == 3) girarIzquierda();
  if (giro == 2) mediaVuelta();
  if (robot.fallo) { fase = 3; detener(); return; }
  direccion = nueva;
  if (giro != 0 && !actualizarSensores()) return;
  if (paredFrente()) { fase = 3; detener(); return; }
  registrar(nueva, false);
  avanzar();
  if (robot.fallo) { fase = 3; detener(); return; }
  // avanzar() es bloqueante. Actualizar odometría solo tras completar el movimiento.
  fila = siguiente / columnas;
  columna = siguiente % columnas;
  movimientosAlgoritmo++;
}

void CiclicoCurvo::inicializarMapa(bool leerAhora) {
  filas = filasLaberinto(); columnas = columnasLaberinto(); cantidad = filas * columnas;
  if (filas < 2 || columnas < 2 || cantidad > CAPACIDAD) { fase = 3; detener(); return; }
  inicio = filaInicial() * columnas + columnaInicial(); centro = metaCentral();
  fila = filaActual(); columna = columnaActual(); direccion = direccionActual();
  reinicioFueraDelInicio = !enInicio();
  fase = 0; observadas = 0; planificaciones = 0; movimientosAlgoritmo = 0;
  indiceRuta = -1; nivel = 0; mapaCambio = true; soloConocidos = false; destinoInicio = false;
  for (int i = 0; i < cantidad; i++) {
    muros[i] = 0; conocidos[i] = 0; visitadas[i] = false; cerrados[i] = false;
    distancias[i] = INFINITO; padres[i] = 0; trabajo[i] = 0; ruta[i] = 0;
    int r = i / columnas; int c = i % columnas;
    if (r == 0) { muros[i] = muros[i] | 1; conocidos[i] = conocidos[i] | 1; }
    if (c == columnas - 1) { muros[i] = muros[i] | 2; conocidos[i] = conocidos[i] | 2; }
    if (r == filas - 1) { muros[i] = muros[i] | 4; conocidos[i] = conocidos[i] | 4; }
    if (c == 0) { muros[i] = muros[i] | 8; conocidos[i] = conocidos[i] | 8; }
  }
  if (!leerAhora) {
    trabajo[0] = celdaActual();
    return;
  }
  observar();
  if (robot.fallo || fase == 3) return;
  // Leer el cuarto lado si aún no se conoce, también con orientación inicial cambiada.
  // Dos giros de 180° devuelven la orientación original, sin consultar paredes ocultas.
  if ((conocidos[celdaActual()] & (1 << ((direccion + 2) % 4))) == 0) {
    mediaVuelta();
    if (robot.fallo) { fase = 3; detener(); return; }
    direccion = (direccion + 2) % 4;
    observar();
    if (robot.fallo || fase == 3) return;
    mediaVuelta();
    if (robot.fallo) { fase = 3; detener(); return; }
    direccion = (direccion + 2) % 4;
  }
  trabajo[0] = celdaActual();
}

void CiclicoCurvo::sincronizarRecolocacion() {
  if (enInicio() && celdaActual() != inicio) {
    fila = filaInicial(); columna = columnaInicial(); direccion = direccionActual();
    indiceRuta = -1; mapaCambio = true;
    // Conserva el mapa pero reanuda planificación de la fase vigente.
  }
}

float CiclicoCurvo::costoHeuristica(int estado) {
  if (TIPO == 0) return heuristica(estado / 4);
  return heuristica(estado / 4) * pasoCentimetros() / velocidadMaxima();
}

float CiclicoCurvo::costoArista(int cuartos, int celdas) {
  if (TIPO == 0) return celdas + 3 * cuartos;
  return tiempoRecta(celdas) + tiempoGiro(cuartos) + penalizacionGiro * cuartos;
}

void CiclicoCurvo::ponerEnHeap(int estado) {
  claves[estado] = distancias[estado] + costoHeuristica(estado);
  int i = posiciones[estado];
  if (i < 0) { i = abiertosCantidad; heap[i] = estado; posiciones[estado] = i; abiertosCantidad++; }
  while (i > 0) {
    int p = (i - 1) / 2;
    if (claves[heap[p]] <= claves[estado]) break;
    heap[i] = heap[p]; posiciones[heap[i]] = i; i = p;
  }
  heap[i] = estado; posiciones[estado] = i;
}

int CiclicoCurvo::sacarHeap() {
  if (abiertosCantidad == 0) return -1;
  int elegido = heap[0]; posiciones[elegido] = -1; abiertosCantidad--;
  if (abiertosCantidad == 0) return elegido;
  int ultimo = heap[abiertosCantidad]; int i = 0;
  while (i * 2 + 1 < abiertosCantidad) {
    int hijo = i * 2 + 1;
    if (hijo + 1 < abiertosCantidad && claves[heap[hijo+1]] < claves[heap[hijo]]) hijo++;
    if (claves[ultimo] <= claves[heap[hijo]]) break;
    heap[i] = heap[hijo]; posiciones[heap[i]] = i; i = hijo;
  }
  heap[i] = ultimo; posiciones[ultimo] = i;
  return elegido;
}

void CiclicoCurvo::planificarBusqueda() {
  planificaciones++;
  int origen = celdaActual() * 4 + direccion;
  int estados = cantidad * 4;
  longitudRuta = 0; indiceRuta = -1; abiertosCantidad = 0;
  origenPlan = origen / 4; rumboPlan = origen % 4;
  for (int i = 0; i < estados; i++) { distancias[i] = INFINITO; cerrados[i] = false; posiciones[i] = -1; }
  distancias[origen] = 0; ponerEnHeap(origen);
  int destino = -1;
  for (int iteracion = 0; iteracion < estados; iteracion++) {
    int elegido = sacarHeap();
    if (elegido < 0) break;
    if (esDestino(elegido / 4)) { destino = elegido; break; }
    cerrados[elegido] = true;
    int celda = elegido / 4; int rumbo = elegido % 4;
    for (int dir = 0; dir < 4; dir++) {
      int giro = (dir - rumbo + 4) % 4; if (giro == 3) giro = 1;
      int siguiente = celda;
      int limite = 1; if (TIPO > 0) limite = cantidad;
      for (int largo = 1; largo <= limite; largo++) {
        if (!pasoDisponible(siguiente,dir)) break;
        siguiente = vecino(siguiente,dir); int estado = siguiente * 4 + dir;
        float coste = distancias[elegido] + costoArista(giro,largo);
        if (!cerrados[estado] && coste < distancias[estado]) {
          distancias[estado] = coste; padres[estado] = elegido; ponerEnHeap(estado);
        }
      }
    }
  }
  if (destino < 0) { fase = 3; detener(); return; }
  costoPlan = distancias[destino];
  int actual = destino;
  while (actual != origen && longitudRuta < estados) {
    int padre = padres[actual]; int celda = actual / 4;
    int atras = (actual % 4 + 2) % 4;
    while (celda != padre / 4 && longitudRuta < estados) {
      ruta[longitudRuta] = celda; longitudRuta++; celda = vecino(celda,atras);
    }
    actual = padre;
  }
  if (actual != origen) { fase = 3; detener(); return; }
  indiceRuta = longitudRuta - 1; mapaCambio = false;
}

void CiclicoCurvo::llenarDistancias(int modo) {
  for (int i = 0; i < cantidad; i++) distMapa[i] = INFINITO;
  int cabeza = 0; int cola = 0;
  if (modo == 2) {
    for (int i = 0; i < cantidad; i++) {
      if (esMeta(i)) { distMapa[i] = 0; trabajo[cola] = i; cola++; }
    }
  } else {
    int origen = inicio; if (modo == 0) origen = celdaActual();
    distMapa[origen] = 0; trabajo[cola] = origen; cola++;
  }
  bool anterior = soloConocidos; soloConocidos = modo == 0 || modo == 3;
  while (cabeza < cola) {
    int u = trabajo[cabeza]; cabeza++;
    for (int dir = 0; dir < 4; dir++) {
      if (pasoDisponible(u,dir)) {
        int v = vecino(u,dir);
        if (distMapa[v] == INFINITO) { distMapa[v] = distMapa[u] + 1; trabajo[cola] = v; cola++; }
      }
    }
  }
  soloConocidos = anterior;
}

void CiclicoCurvo::elegirFrontera() {
  objetivoExploracion = -1;
  if (desviosRetorno >= maxDesvios) return;
  llenarDistancias(0); for (int i = 0; i < cantidad; i++) distActual[i] = distMapa[i];
  llenarDistancias(1); for (int i = 0; i < cantidad; i++) distInicioOpt[i] = distMapa[i];
  llenarDistancias(2); for (int i = 0; i < cantidad; i++) distMetaOpt[i] = distMapa[i];
  int mejor = INFINITO;
  for (int i = 0; i < cantidad; i++) {
    // Fronteras accesibles por pasos confirmados; desconocido nunca equivale a seguro.
    // El potencial ordena, no descarta: una ruta larga puede permitir mayor velocidad.
    if (!visitadas[i] && distActual[i] < INFINITO && conocidos[i] != 15 &&
        distInicioOpt[i] < INFINITO && distMetaOpt[i] < INFINITO) {
      int potencial = distInicioOpt[i] + distMetaOpt[i];
      int valor = potencial * 4 + distActual[i];
      if (valor < mejor) { mejor = valor; objetivoExploracion = i; }
    }
  }
  if (objetivoExploracion >= 0) { desviosRetorno++; desviosTotales++; }
}

void CiclicoCurvo::prepararCandidata() {
  prepararRuta(TIPO - 1);
  for (int i = 0; i < cantidad; i++) mapaRuta(i,muros[i],conocidos[i]);
  puntoRuta(celdaActual());
  for (int i = indiceRuta; i >= 0; i--) puntoRuta(ruta[i]);
}

void CiclicoCurvo::prepararMejorGuardada() {
  prepararRuta(TIPO - 1);
  for (int i = 0; i < cantidad; i++) mapaRuta(i,muros[i],conocidos[i]);
  puntoRuta(celdaActual());
  for (int i = longitudMejor - 1; i >= 0; i--) puntoRuta(mejorRuta[i]);
}

void CiclicoCurvo::elegirCarreraRapida() {
  prepararCandidata();
  float nueva = tiempoRutaEstimada();
  float anterior = INFINITO;
  if (longitudMejor > 0) { prepararMejorGuardada(); anterior = tiempoRutaEstimada(); }
  if (nueva <= anterior) {
    longitudMejor = longitudRuta;
    for (int i = 0; i < longitudRuta; i++) mejorRuta[i] = ruta[i];
    prepararCandidata(); tiempoMejor = nueva;
  } else tiempoMejor = anterior;
  trayectoriaPreparada = true;
}

void CiclicoCurvo::comenzarRetorno() {
  fase = 1; destinoInicio = true; soloConocidos = !reinicioFueraDelInicio;
  trayectoriaPreparada = false; objetivoExploracion = -1;
  desviosRetorno = 0; mapaCambio = true; indiceRuta = -1;
  elegirFrontera();
}

void CiclicoCurvo::seguirBusqueda() {
  if (!trayectoriaPreparada && (mapaCambio || indiceRuta < 0)) planificarBusqueda();
  if (fase == 3) return;
  if (fase == 2) {
    if (!trayectoriaPreparada) elegirCarreraRapida();
    if (!rutaTerminada()) seguirRuta();
    if (robot.fallo || fase == 3) { fase = 3; detener(); return; }
    fila = filaActual(); columna = columnaActual(); direccion = direccionActual();
    if (rutaTerminada()) {
      carrerasCompletadas++;
      if (carrerasCompletadas >= maxCarreras) { fase = 3; detener(); }
      else comenzarRetorno();
    }
    return;
  }
  if (indiceRuta >= 0) {
    const int destino = ruta[indiceRuta];
    moverHacia(destino);
    if (celdaActual() == destino) indiceRuta--;
  }
}

void CiclicoCurvo::actualizarFases() {
  if (fase == 0 && esMeta(celdaActual())) {
    carrerasCompletadas++;
    if (carrerasCompletadas >= maxCarreras) { fase = 3; detener(); return; }
    comenzarRetorno();
  }
  if (fase == 1 && objetivoExploracion >= 0 && celdaActual() == objetivoExploracion) {
    objetivoExploracion = -1; mapaCambio = true; indiceRuta = -1;
  }
  // Observar durante el regreso puede revelar fronteras que no existían al salir del centro.
  if (fase == 1 && objetivoExploracion < 0 && (mapaCambio || enInicio())) {
    elegirFrontera();
    if (objetivoExploracion >= 0) { mapaCambio = true; indiceRuta = -1; }
  }
  // El inicio termina el retorno solo cuando no queda un objetivo activo.
  if (fase == 1 && enInicio() && objetivoExploracion < 0) {
    objetivoExploracion = -1;
    fase = 2; destinoInicio = false; soloConocidos = true; reinicioFueraDelInicio = false;
    trayectoriaPreparada = false; mapaCambio = true; indiceRuta = -1;
  }
}

void CiclicoCurvo::begin() {
  maxCarreras = limiteCarreras(); maxDesvios = presupuestoExploracion();
  carrerasCompletadas = 0; desviosTotales = 0; longitudMejor = 0; tiempoMejor = INFINITO;
  objetivoExploracion = -1; trayectoriaPreparada = false;
  filas = filasLaberinto(); columnas = columnasLaberinto(); cantidad = filas * columnas;
  localizando = localizacionAutomatica;
  patronLocalizacion = -1; capturasLocalizacion = 0;
  paredes.reiniciarFiltro();
  fase = 0;
  if (localizando) {
    // No asignar una esquina ni precargar bordes hasta identificar el interior.
    fila = -1; columna = -1; inicio = -1; direccion = 0;
    if (!localizacion.iniciar(filas, columnas)) {
      fallarLocalizacion(localizacion.error());
      return;
    }
    Serial.println("LOCALIZACION: esquina con pared detras y frente libre; interior aun desconocido.");
    return;
  }
  filaOrigen = MapaSimulador::INICIO_FILA;
  columnaOrigen = MapaSimulador::INICIO_COLUMNA;
  fila = filaOrigen; columna = columnaOrigen; direccion = MapaSimulador::INICIO_RUMBO;
  inicializarMapa();
}

void CiclicoCurvo::step() {
  if (fase == 3) { detener(); return; }
  if (localizando) { pasoLocalizacion(); return; }
  // Avisar la llegada aunque los sensores aun esten adquiriendo la nueva posicion.
  if (fase == 0 && esMeta(celdaActual())) { observar(); actualizarFases(); return; }
  sincronizarRecolocacion(); if (!observar()) return;
  if (robot.fallo || fase == 3) return;
  const int llegadasAntes = carrerasCompletadas;
  actualizarFases();
  if (fase == 3 || carrerasCompletadas != llegadasAntes) return;
  seguirBusqueda();
}

int CiclicoCurvo::filasLaberinto() { return MapaSimulador::FILAS; }

int CiclicoCurvo::columnasLaberinto() { return MapaSimulador::COLUMNAS; }

int CiclicoCurvo::filaInicial() { return filaOrigen; }

int CiclicoCurvo::columnaInicial() { return columnaOrigen; }

int CiclicoCurvo::filaActual() { return fila; }

int CiclicoCurvo::columnaActual() { return columna; }

int CiclicoCurvo::direccionActual() { return direccion; }

bool CiclicoCurvo::metaCentral() { return true; }

bool CiclicoCurvo::enInicio() { return celdaActual() == inicio; }

int CiclicoCurvo::limiteCarreras() { return ConfiguracionUnificada::LLEGADAS; }

int CiclicoCurvo::presupuestoExploracion() { return ConfiguracionUnificada::ZONAS_POR_RETORNO; }

float CiclicoCurvo::pasoCentimetros() { return robot.CELDA * 100; }

float CiclicoCurvo::velocidadMaxima() { return robot.V_MAX * 100; }

float CiclicoCurvo::duracion(float d, float vmax, float acc, float vmin) {
    if (d <= 0) return 0;
    float pico = fminf(vmax, sqrtf(acc*d+vmin*vmin));
    float ta = (pico-vmin)/acc;
    float da = (pico*pico-vmin*vmin)/(2*acc);
    return 2*ta + fmaxf(0, d-2*da)/pico;
}

float CiclicoCurvo::tiempoRecta(int n) { return duracion(n*robot.CELDA,robot.V_MAX,robot.A_MAX,robot.V_MIN); }

float CiclicoCurvo::tiempoGiro(int q) { return q ? duracion(q*PI/2,robot.W_MAX,robot.ALFA_MAX,robot.W_MIN):0; }

bool CiclicoCurvo::paredFrente() { return paredes.leer(0,fila,columna,direccion); }

bool CiclicoCurvo::paredIzquierda() { return paredes.leer(1,fila,columna,direccion); }

bool CiclicoCurvo::paredDerecha() { return paredes.leer(2,fila,columna,direccion); }

void CiclicoCurvo::girarDerecha() {
  if (!robot.ejecutar('G', -90)) robot.fallo = true;
  if (!robot.fallo) paredes.reiniciarFiltro();
}

void CiclicoCurvo::girarIzquierda() {
  if (!robot.ejecutar('G', 90)) robot.fallo = true;
  if (!robot.fallo) paredes.reiniciarFiltro();
}

void CiclicoCurvo::mediaVuelta() {
  if (!robot.ejecutar('G', 180)) robot.fallo = true;
  if (!robot.fallo) paredes.reiniciarFiltro();
}

void CiclicoCurvo::avanzar() {
  if (!robot.ejecutar('A', 1)) robot.fallo = true;
  if (!robot.fallo) paredes.reiniciarFiltro();
}

void CiclicoCurvo::detener() { robot.stop(); }

void CiclicoCurvo::prepararRuta(int) {
    // La variante original pide modo 2 (curvas/diagonales). Se conserva la ruta
    // confirmada y se ejecuta con A/G, las acciones disponibles en este control.
    siguienteTramo=1; puntos=0; terminada=false;
}

void CiclicoCurvo::mapaRuta(int c,int m,int k) { rutaMuros[c]=m; rutaConocidos[c]=k; }

void CiclicoCurvo::puntoRuta(int c) { if (puntos<1025) trayectoria[puntos++]=c; else robot.fallo=true; }

bool CiclicoCurvo::rutaTerminada() { return terminada; }

int CiclicoCurvo::rumboEntre(int a,int b) { for(int d=0;d<4;d++) if(vecino(a,d)==b) return d; return -1; }

float CiclicoCurvo::tiempoRutaEstimada() {
    float t=0; int rumbo=direccion;
    for(int i=1;i<puntos;) {
        int d=rumboEntre(trayectoria[i-1],trayectoria[i]);
        if(d<0) return INFINITO;
        int q=(d-rumbo+4)%4; if(q==3) q=1;
        int n=1; while(i+n<puntos && rumboEntre(trayectoria[i+n-1],trayectoria[i+n])==d) n++;
        t+=tiempoGiro(q)+tiempoRecta(n);
        rumbo=d; i+=n;
    } return t;
}

void CiclicoCurvo::seguirRuta() {
    for(int i=siguienteTramo;i<puntos && !robot.fallo;) {
        int d=rumboEntre(trayectoria[i-1],trayectoria[i]);
        if(d<0) { robot.fallo=true; break; }
        int n=1; while(i+n<puntos && rumboEntre(trayectoria[i+n-1],trayectoria[i+n])==d) n++;
        int c=celdaActual();
        for(int k=0;k<n;k++) {
            int v=vecino(c,d), bit=1<<d, op=1<<((d+2)%4);
            if(v<0 || !(rutaConocidos[c]&bit) || !(rutaConocidos[v]&op) ||
                (rutaMuros[c]&bit) || (rutaMuros[v]&op)) { robot.fallo=true; break; }
            c=v;
        }
        if(robot.fallo) break;
        int q=(d-direccion+4)%4;
        if(q==1) girarDerecha(); else if(q==3) girarIzquierda(); else if(q==2) mediaVuelta();
        if(robot.fallo) break;
        direccion=d;
        if (!actualizarSensores()) return;
        if(paredFrente()) { robot.fallo=true; break; }
        if(!robot.ejecutar('A',n)) { robot.fallo=true; break; }
        paredes.reiniciarFiltro();
        direccion=d; fila=c/columnas; columna=c%columnas; movimientosAlgoritmo+=n; i+=n;
        siguienteTramo=i; terminada=siguienteTramo>=puntos;
        return;
    }
    terminada=!robot.fallo && siguienteTramo>=puntos;
    if(robot.fallo) { fase=3; detener(); }
}


bool CiclicoCurvo::centroLocalizado() const {
  return !localizacionAutomatica || localizacion.estado() == LocalizacionEsquina::Estado::Localizado;
}

void CiclicoCurvo::fallarLocalizacion(const char* motivo) {
  Serial.print("ERROR LOCALIZACION: "); Serial.println(motivo);
  robot.fallo = true; fase = 3; detener();
}

void CiclicoCurvo::pasoLocalizacion() {
  // Acumular tres capturas nuevas entre vueltas de loop(), sin esperar aqui.
  if (!actualizarSensores() || !paredes.capturaNueva()) return;
  const int patron = (paredes.leer(0, 0, 0, 0) ? 1 : 0) |
                     (paredes.leer(1, 0, 0, 0) ? 2 : 0) |
                     (paredes.leer(2, 0, 0, 0) ? 4 : 0);
  if (patron != patronLocalizacion) {
    patronLocalizacion = patron;
    capturasLocalizacion = 1;
  } else ++capturasLocalizacion;
  if (capturasLocalizacion < 3) return;
  capturasLocalizacion = 0;
  const auto estado = localizacion.observar(patron & 1, patron & 2, patron & 4);
  if (estado == LocalizacionEsquina::Estado::Error) {
    fallarLocalizacion(localizacion.error());
    return;
  }
  if (estado == LocalizacionEsquina::Estado::Localizado) {
    finalizarLocalizacion();
    return;
  }
  Serial.println("LOCALIZACION: ambos lados cerrados; avanzar una casilla por el borde.");
  if (!robot.ejecutar('A', 1) || robot.fallo) {
    fallarLocalizacion("Avance no completado; no se actualiza la posicion.");
    return;
  }
  if (!localizacion.confirmarAvance()) {
    fallarLocalizacion("No se pudo confirmar el avance inicial.");
    return;
  }
  paredes.reiniciarFiltro();
}

void CiclicoCurvo::finalizarLocalizacion() {
  filaOrigen = localizacion.filaInicio();
  columnaOrigen = localizacion.columnaInicio();
  fila = localizacion.filaActual(); columna = columnaOrigen; direccion = 0;
  // Crear los bordes solo despues de identificar el lado interior.
  inicializarMapa(false);
  if (fase == 3) return;
  for (int paso = 0; paso <= localizacion.pasos(); ++paso) {
    fila = filaOrigen - paso;
    const auto& dato = localizacion.observacion(paso);
    registrar(0, dato.frente);
    registrar(3, dato.izquierda);
    registrar(1, dato.derecha);
    visitadas[celdaActual()] = true;
    ++observadas;
  }
  movimientosAlgoritmo = localizacion.pasos();
  reinicioFueraDelInicio = false;
  localizando = false;
  Serial.print("LOCALIZADO: interior a la ");
  Serial.println(localizacion.interior() == LocalizacionEsquina::Interior::Derecha ? "DERECHA" : "IZQUIERDA");
  Serial.print("Inicio propio fila,columna: "); Serial.print(filaOrigen);
  Serial.print(","); Serial.println(columnaOrigen);
  Serial.print("Posicion actual fila,columna: "); Serial.print(fila);
  Serial.print(","); Serial.println(columna);
  Serial.print("Centro: filas "); Serial.print((filas - 1) / 2);
  Serial.print(".."); Serial.print(filas / 2);
  Serial.print("; columnas "); Serial.print((columnas - 1) / 2);
  Serial.print(".."); Serial.println(columnas / 2);
}
