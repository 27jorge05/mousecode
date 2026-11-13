# Unificación: cíclico recto + control del ESP32

## Prueba actual: sensores y órdenes sin motores

El monitor a 921600 baudios muestra las tres distancias aproximadamente cada 500 ms. `INICIAR` consulta al algoritmo y muestra la primera orden que pediría: avanzar, girar izquierda/derecha o media vuelta. Esa orden no se ejecuta ni se confirma como completada: se detiene el intento para no inventar una posición nueva. Las distancias siguen apareciendo; reinicia el ESP32 para repetir el intento.

El cuerpo original de `ControlMovimiento::ejecutar()` está conservado dentro de un comentario de bloque en `src/control/ControlMovimiento.cpp`. La inicialización del control/IMU también está comentada en `src/main.cpp`: esta prueba requiere únicamente los sensores. Las secciones siguientes describen el funcionamiento con el control real, actualmente desactivado.

Proyecto independiente, listo para compilar. Incluye el navegador cíclico de rectas, el control recibido y sus bibliotecas locales. No necesitas copiar archivos de `control/` ni modificar el simulador. La carpeta se llama `unificacacion` para que coincida con el nombre solicitado y con el sketch de Arduino.

**Unionfinal ahora usa tres VL53L0X reales para detectar paredes.** Izquierda/frontal/derecha están a 90°, con XSHUT D14/D27/D32 y SDA D25/SCL D26 compartidos con la IMU. La clase `SensoresParedHardcodeados` permanece para pruebas, pero `main.cpp` construye `SensoresParedVL53L0X`. Los algoritmos solo reciben las mediciones de su posición actual; no cargan el vector de paredes del simulador. Dimensiones, inicio y meta siguen siendo datos conocidos del laberinto. La pose se estima con encoders/IMU: el robot debe comenzar en el inicio y rumbo configurados.

## Calibracion persistente

`CALIBRAR minimo_cm maximo_cm` captura 5 segundos por sensor, con dos parpadeos antes de cada uno. Cuando las tres capturas son validas, sus pendientes y desplazamientos se guardan juntos en flash con Preferences. Al volver a encender el ESP32 se cargan automaticamente y Serial muestra `Calibracion cargada desde flash: IZQ / FRONTAL / DER.`. Si falla una captura o el guardado, se conserva el ajuste anterior. Si no hay un registro compatible, se usan los polinomios originales. Los filtros EMA se reinician al cargar; solo se guardan los coeficientes, no las lecturas anteriores. No hace falta repetir la calibracion al apagar y encender. Los motores continuan desactivados en el modo de prueba actual.

## Localizacion automatica desde una esquina

En `src/main.cpp`, `USAR_MAPA_SIMULADO = false` selecciona los VL53L0X y activa la localizacion inicial en ambos algoritmos. Con `true`, se consulta el vector de referencia y se usa su esquina/rumbo configurados. La bandera cambia la fuente de paredes; no activa ni simula el control de motores. El toggle `MOSTRARSENSORES` solo controla las lecturas de diagnostico.

Para sensores, el carrito debe estar centrado en una casilla de esquina, con una pared detras y apuntando hacia una salida libre. La pared trasera es una condicion de colocacion: los tres sensores delanteros no la comprueban. Saber que hay dos paredes en cualquier otra casilla no demuestra que sean bordes exteriores.

- Dos paredes (la trasera y una lateral): la abertura lateral identifica inmediatamente el lado interior.
- Tres paredes: avanza una casilla por vez, sin girar, hasta encontrar la primera abertura lateral. La otra lateral corresponde al borde.
- Se requieren tres capturas coherentes por observacion. Frente bloqueado al iniciar, ambos lados abiertos durante esta fase, lecturas invalidas/inestables, encierro o exceso de longitud detienen el intento.
- Conocer el lado interior fija unas coordenadas propias: salida inicial hacia norte; inicio abajo a la izquierda si el interior queda a la derecha, o abajo a la derecha en el caso opuesto. No identifica el norte geografico ni el nombre fisico absoluto de la esquina.
- La posicion actual descuenta las casillas ya avanzadas. El centro puede quedar detras si la abertura aparece tarde. Se conserva el corredor observado, incluidas sus conexiones, para explorar y regresar al inicio.
- Despues de localizar, A* y los ciclos existentes siguen sobre el mapa aprendido. Los cruces posteriores con ambos laterales abiertos son validos.

`LocalizacionEsquina.h/.cpp` contiene la deduccion geometrica comun. La localizacion admite laberintos cuadrados de 2 a 16 casillas por lado, conforme al limite de 256 celdas de los algoritmos. Un rectangulo requiere distinguir ademas sus ejes; esta version lo rechaza. Las coordenadas del centro son las casillas centrales calculadas desde esas dimensiones conocidas, sin consultar paredes ocultas.

El EMA se reinicia tras un desplazamiento o giro completado para no mezclar paredes de distintas posiciones/orientaciones. Una orden de movimiento que no termina correctamente no actualiza la pose ni los avances de localizacion. Mientras siga activo el modo sin motores en ControlMovimiento::ejecutar, la primera orden solo se imprime y detiene el intento; no se inventa movimiento para completar la localizacion.

Validacion: compilacion ESP32 y pruebas nativas con cuatro esquinas, dos rumbos de salida por esquina, aberturas en pasos 0/2/15 y ambos algoritmos. Incluyen los ciclos de exploracion-retorno-carrera, errores de lectura/movimiento, limites del borde, ambos modos de fuente y parada. No se hicieron pruebas fisicas ni se cargo firmware al ESP32.

## Organización

La estructura sigue el estándar nativo de PlatformIO. El punto de entrada principal del firmware es `src/main.cpp`, junto con los módulos y componentes organizados en subcarpetas de `src/`. Se incluye también la estructura de directorios `include/` y `lib/` requerida por proyectos PlatformIO. Cada clase de control, navegación y sensores cuenta con su declaración en `.h` y su implementación en `.cpp`. Los archivos de configuración y el vector del mapa residen como headers de constantes.

```text
unificacacion/
├── platformio.ini
├── include/
├── lib/
├── exportar_mapa.py
├── micromouse-centro.json
└── src/
    ├── main.cpp        # Punto de entrada (setup, loop, serial)
    ├── algoritmos/     # CiclicoRecto
    ├── control/        # ControlMovimiento
    ├── giroscopio/     # FusionAngular y MPU6050
    ├── configuracion/ # Ciclos y velocidades
    ├── mapa/          # Vector de paredes
    ├── sensores/      # Lectura VL53L0X y fuente simulada para pruebas
    ├── motor/
    ├── digitalpin/
    ├── pwm/
    ├── codificador/
    ├── controlpi/
    ├── lazop/
    ├── mezclador/
    ├── odometria/
    └── perfil/
```

## Qué abrir y cargar

### PlatformIO — estructura estándar

Abre **esta carpeta** en VS Code con PlatformIO. El destino es `esp32dev`, framework Arduino, plataforma `espressif32@6.9.0`, biblioteca `ESP32Encoder@0.12.0`.

Desde la raíz del proyecto:

```bash
pio run
pio run -t upload
pio device monitor -b 921600
```

Si hay más de un puerto, añade `--upload-port /dev/ttyUSB0` al comando de carga; cambia el puerto por el de tu equipo. En Windows puede ser `COM5`. Para elegir el puerto del monitor, usa `pio device monitor -p /dev/ttyUSB0 -b 921600`.

## Arranque

1. Al encender imprime dimensiones, inicio, rumbo y ciclos. El encabezado indica **PAREDES REALES: VL53L0X IZQ / FRONTAL / DER**. Inicializa IMU y después los sensores, en ese orden, sobre el mismo bus I²C.
2. Mantén el robot quieto mientras calibra el MPU6050. Si falla la IMU o el mapa es inválido, no permite arrancar.
3. Colócalo centrado en la casilla inicial, apuntando hacia su única salida. El mapa incluido usa fila **15**, columna **0**, rumbo **norte (0)**. Los índices empiezan en cero.
4. Puedes iniciar el recorrido de dos formas:
   - **Botón físico**: Pulsa el botón `BOOT` (GPIO 0) de la placa ESP32 (incluye 1 segundo de cortesía para retirar la mano).
   - **Serial**: En el monitor a **921600 baudios**, escribe `INICIAR` y pulsa Enter.
   No arranca automáticamente al encender para evitar movimientos inesperados.
5. Completa por defecto **2,5 ciclos**: inicio → centro → inicio → centro → inicio → centro.
6. `!` detiene y bloquea la ejecución. Durante movimientos el control también atiende este carácter. Después de terminar o abortar, reinicia el ESP32 y recoloca el robot para repetir. `MAPA` + Enter vuelve a imprimir el vector cuando se atiende el bucle principal. Durante una acción de movimiento solo se acepta `!`; los demás caracteres se descartan. Se aceptan Enter con LF, CR o CRLF.

El control usa rectas continuas en las carreras confirmadas y pivotes; no hace slalom ni diagonales. Actualiza la casilla y el rumbo únicamente después de confirmar una acción. Ante timeout, fallo de IMU o parada, corta los motores.

## Mapa de referencia para dimensiones y pruebas

Se copió exactamente [micromouse-centro.json](micromouse-centro.json) desde `laberintos/micromouse-centro.json` del simulador: **16×16, casillas de 18 cm, semilla 2026, meta central**. No se inventó un mapa alternativo. El firmware no se comunica en vivo con el navegador: usa la instantánea convertida al compilar.

El vector de referencia está en [MapaSimulador.h](src/mapa/MapaSimulador.h), pero la fuente física seleccionada no lo consulta. Cambiarlo no cambia las paredes reales que ve el carrito:

```cpp
constexpr uint8_t PAREDES[] = {
    // Una máscara por casilla, ordenada por filas.
};
```

La declaración real contiene los 256 valores. `indice = fila * COLUMNAS + columna`.

| Pared | Bit |
|---|---:|
| Norte | 1 |
| Este | 2 |
| Sur | 4 |
| Oeste | 8 |

Suma los bits de las paredes presentes: `9 = norte + oeste`, `15 = cerrada por los cuatro lados`, `0 = sin paredes`. Si añades una pared este, añade también la pared oeste en la casilla vecina. El borde exterior debe permanecer cerrado. La orientación inicial usa norte=0, este=1, sur=2, oeste=3; estos son rumbos, no máscaras de pared.

Puedes editar directamente `PAREDES`, dimensiones, inicio y rumbo del header. Es un arreglo de tamaño fijo para evitar asignaciones dinámicas; cada cambio requiere recompilar y volver a cargar. Al hacerlo, el SHA del JSON queda como procedencia del archivo original, no como certificado del vector manual. El validador de arranque revisa máscaras, bordes, simetría e inicio. Para verificar además que el centro sea accesible, utiliza la conversión del JSON descrita abajo.

### Usar otro mapa guardado en el simulador

Guarda el laberinto en el simulador y ejecuta desde la raíz:

```bash
python3 unificacacion/exportar_mapa.py laberintos/mi-laberinto.json
pio run -d unificacacion -t upload
```

El exportador sustituye `src/mapa/MapaSimulador.h` con los valores de ese JSON. **Sobrescribe cualquier edición manual anterior del vector.** No altera el JSON ni el mapa del simulador. El nombre y hash impresos corresponden al JSON usado en la generación.

Admite mapas rectangulares con **hasta 256 casillas**: no está limitado a 16×16, pero un mapa 50×50 excede la capacidad de este navegador embebido. Exige meta central, paredes simétricas, borde cerrado, centro accesible e inicio con una única salida. Al cambiar dimensiones se regeneran también fila, columna, orientación y tamaño físico de casilla.

Para comprobar la copia original incluida:

```bash
python3 unificacacion/exportar_mapa.py --check
```

Si generaste desde otro JSON, pásalo también al comprobador: `python3 unificacacion/exportar_mapa.py laberintos/mi-laberinto.json --check`. Esto compara el header con el JSON, por lo que una edición manual distinta produce un aviso de desincronización.

## Configuración rápida

[ConfiguracionUnificada.h](src/configuracion/ConfiguracionUnificada.h) contiene ciclos y límites de movimiento:

| Parámetro | Valor inicial |
|---|---|
| Ciclos | 2.5; hasta 15.5, en pasos de 1 |
| Zonas por retorno | 8; de 0 a 32, 0 regresa directamente |
| Velocidad máxima | 0.50 m/s |
| Aceleración | 1.00 m/s² |
| Velocidad mínima del perfil | 0.10 m/s |
| Giro máximo | 400°/s |
| Aceleración angular | 1000°/s² |

El tamaño de casilla viene de `src/mapa/MapaSimulador.h` y se convierte de cm a metros para el control. Estos límites son consignas heredadas del control, no una calibración nueva ni una garantía de precisión en el robot físico.

## Pines conservados

| Elemento | Pines / valor |
|---|---|
| Motor izquierdo IN1 / IN2 / PWM | 5 / 18 / 22 |
| Encoder izquierdo A / B | 4 / 34 |
| Motor derecho IN1 / IN2 / PWM | 19 / 21 / 23 |
| Encoder derecho A / B | 35 / 13 |
| MPU6050 SDA / SCL | 25 / 26 |
| Radio de rueda / base | 0.02205 m / 0.088 m |
| Encoder PPR / RPM máxima | 200 / 450 |

Las ganancias PI se conservan: izquierda Kp=0.1396, Ki=1.4697; derecha Kp=0.1330, Ki=1.4276. No se asignan pines IR porque no existen esas lecturas en el control recibido.

## Archivos y responsabilidades

- `src/main.cpp`: composición de objetos, calibración, comandos y bucle principal.
- `src/algoritmos/CiclicoRecto.h` / `CiclicoRecto.cpp`: algoritmo y adaptador de movimientos; conserva mapa aprendido separado del vector usado como sensor.
- `src/sensores/SensoresPared.h` / `SensoresPared.cpp`: contrato `FuenteParedes` y `SensoresParedHardcodeados`. Responde frente/izquierda/derecha desde la pose confirmada, sin precargar el mapa aprendido.
- `src/control/ControlMovimiento.h` / `ControlMovimiento.cpp`: control recibido encapsulado: perfiles, lazos, mezclador, encoders y fusión. Movimiento bloqueante con finalización por tolerancias y fallo por timeout.
- Bibliotecas `.h`/`.cpp`: copias locales de la adaptación previa del control; incluidas para compilar esta carpeta sola.
- `exportar_mapa.py`, `micromouse-centro.json` y `src/mapa/MapaSimulador.h`: procedencia y conversión de las paredes hardcodeadas.

La fusión usa giro positivo a la izquierda. El navegador enumera rumbos en sentido horario: girar a la derecha se traduce a −90° en el control. La carrera rápida actualiza ese rumbo antes de consultar de nuevo la pared frontal.

No se alteró `control/control def`, el algoritmo del simulador ni el ZIP. Se reutiliza la adaptación anterior de `arduino/CiclicoRectoESP32` con sus protecciones de PI, IMU, timeout y odometría, cambiando aquí la fuente de paredes y la configuración del mapa. `ejecutar()` y `stop()` admiten sustitución para pruebas nativas sin mover hardware.

## Validación reproducible

```bash
python3 tests/unificacion/run.py
```

La prueba revisa las 256 máscaras, todas las orientaciones y lados, separación de mapa aprendido, recorrido completo sin atravesar paredes, tres llegadas/dos retornos y aborto sin confirmar una casilla no recorrida. Usa un doble de movimiento: **no valida motores, respuesta del MPU6050 ni calibración física**. La compilación PlatformIO valida el SDK y el enlazado para ESP32; cargar y ensayar en tu placa es un paso distinto.

## Registro previo de compilación ESP32

Antes de reorganizar las carpetas, el 3 de noviembre de 2026 se compiló y enlazó este proyecto con el SDK real de ESP32: PlatformIO, espressif32 6.9.0, Arduino ESP32 2.0.17 y ESP32Encoder 0.12.0. Resultado **SUCCESS**; RAM estática 50.660 / 327.680 bytes (15,5 %) y programa 317.797 / 1.310.720 bytes (24,2 %). Esto no mide el máximo uso de pila durante la ejecución ni sustituye una prueba física.

El archivo de aplicación generado está en `.pio/build/esp32dev/firmware.bin`. Utiliza **Upload** de PlatformIO para cargar el conjunto de archivos con los offsets de la placa. No se ha cargado ni ejecutado en hardware desde esta sesión.

Referencias oficiales de configuración: [ESP32 Dev Module](https://docs.platformio.org/en/latest/boards/espressif32/esp32dev.html) y [directorio de fuentes de PlatformIO](https://docs.platformio.org/en/latest/projectconf/sections/platformio/options/directory/src_dir.html).

La reorganización en `src/` pasó las pruebas nativas de mapa, sketch y recorridos. El registro de SDK anterior no certifica esta revisión: falta repetir `pio run -d unificacacion` con PlatformIO instalado.

## Diagnóstico de arranque y monitor serie (Fab)

El firmware usa **921600 baudios**, igual que `monitor_speed` en `platformio.ini`. Selecciona el puerto de la placa que cargaste y terminación de línea LF, CR o CRLF. Esta revisión acepta `INICIAR` o `iniciar`, y elimina espacios externos. `AYUDA` muestra comandos, `ESTADO` imprime el estado actual y `MAPA` imprime dimensiones e inicio; `SENSORES` muestra distancias e identificación. Los comandos incorrectos o demasiado largos ahora responden con un aviso.

Hay mensajes `FAB ARRANQUE` antes de construir el control y marcadores al inicializar encoders e IMU. Una vez terminado `setup()`, mientras no haya recorrido activo se imprime el estado cada cinco segundos para poder abrir el monitor después del encendido. Si el programa se bloquea dentro de la inicialización, ese último marcador permite ubicar la etapa; el estado periódico solo funciona cuando ya se llega a `loop()`.

Para probar: recompila y vuelve a cargar **esta carpeta unionfinal** (el ZIP anterior no contiene estos cambios), abre el monitor a 921600, pulsa RESET/EN y envía `ESTADO` con nueva línea. `LISTO` permite `INICIAR` o BOOT; `ERROR_IMU_CONTROL` bloquea el movimiento y requiere revisar el MPU6050 y sus conexiones SDA 25/SCL 26. `!` sigue abortando y exige reiniciar antes de otro recorrido.

Si no aparece nada ni después de RESET, no se puede concluir que sea un fallo del algoritmo: verifica puerto, velocidad, carga efectiva del firmware y conexión serie. Las pruebas nativas verifican inicio, mensajes, comandos, aborto y bloqueo al simular una IMU ausente (`python3 tests/unificacionfab/run.py`). No se ha comprobado esta revisión con SDK ni placa real. No se cambiaron pines, ganancias, mapa ni algoritmo del control.

## Tres sensores reales: pines, coeficientes y prueba

Los dos archivos nuevos son `src/sensores/SensoresParedVL53L0X.h/.cpp`. Los pines, direcciones, coeficientes, umbrales y rango están en el namespace `ConfiguracionVL53L0X` del header.

| ID | Dirección física | XSHUT | Dirección I²C | A | B | C |
|---|---|---|---|---|---|---|
| 1 | Izquierda 90° | D14 | 0x30 | 0.00009978 | 1.018483 | -1.4592 |
| 3 | Frontal | D27 | 0x31 | -0.00209719 | 1.024291 | -2.9011 |
| 5 | Derecha 90° | D32 | 0x32 | -0.00357056 | 1.024716 | -5.3230 |

Se usan las tres fórmulas iniciales del mensaje de calibración, no el juego diferente que aparecía dentro del programa pegado. Primero se convierte `RangeMilliMeter / 10.0f`; después se aplica `corregida_cm = A * cruda_cm² + B * cruda_cm + C`. No se mezclan milímetros y centímetros.

Todos comparten SDA D25 y SCL D26 con el MPU6050. Cada XSHUT es independiente. Al iniciar se mantienen apagados los tres sensores y se activan uno a uno para asignar direcciones distintas. Los canales se identifican por función y por IDs 1,3,5; no se renumeran a 1,2,3.

Las mediciones se capturan con el carrito detenido, antes de observar una casilla y después de girar si hace falta. Son mediciones secuenciales con presupuesto de 80 ms por sensor: un conjunto puede tardar aproximadamente 240 ms o más. Esa pausa se suma al tiempo del recorrido, pero no está dentro del bucle PI de los motores. No hay otra tarea I²C compitiendo con el giroscopio durante una acción.

Se añadió `actualizar()` a la interfaz `FuenteParedes` para capturar un conjunto coherente antes de consultar las tres paredes. La fuente simulada conserva su comportamiento. Ambos algoritmos comprueban el resultado: si una lectura falla, detienen y no registran la captura como paredes ni como espacio libre. También refrescan antes de avanzar tras un giro y entre los tramos de una carrera. No hay detección continua de obstáculos dentro de una recta agrupada; el laberinto se supone estático y el algoritmo usa allí el camino confirmado.

### Cómo comprobarlo en el carrito

1. Compila y sube `unionfinal/unificacacion` con PlatformIO; se añadió `Adafruit VL53L0X@1.2.4` a las dependencias.
2. Abre el monitor a 921600 baudios, con nueva línea, y reinicia manteniendo el carrito quieto.
3. Antes del recorrido envía `SENSORES`. Muestra ID, nombre, XSHUT, distancia cruda, corregida y clasificación PARED/LIBRE/DESCONOCIDO, sin mover motores. El comando solo se atiende parado.
4. Verifica con una regla cada sensor y su orientación. Con una pared delante del sensor 1 debe cambiar IZQ, con la frontal el 3 y con la derecha el 5.
5. Ajusta `UMBRALES_CM` si el montaje lo necesita. Los tres umbrales iniciales son **9 cm**, no una medida de calibración. Toma muestras con el carrito centrado y con/sin pared inmediata para decidir un valor adecuado por sensor.
6. Confirma el rango de calibración; el intervalo de software inicial para lectura cruda es **5..75 cm**, basado en los puntos del calibrador anterior. Las lecturas fuera de ese intervalo, correcciones no positivas o no finitas y errores del sensor se consideran desconocidos. No se recortan a cero ni se interpretan como libre. Un pasillo largo o un objetivo que no devuelve medición también puede detener el recorrido: verifica cómo se comporta el montaje antes de ampliar el rango o cambiar esa política.
7. Si la lectura inicial era inválida, corrige la causa y reinicia: no se permite iniciar mientras el estado sea error. Después de `LISTO`, coloca en el inicio configurado y envía `INICIAR`, o usa BOOT. `!` sigue deteniendo y bloqueando hasta reiniciar.

No se añadieron calibración automática ni movimientos de calibración. Se aplican los coeficientes proporcionados. `CiclicoRecto` sigue activo y `CiclicoCurvo` comentado; ambos usan la misma fuente física al seleccionarlos.

### Verificación de esta revisión

`python3 tests/unionfinal-sensores/run.py` compila con dobles de hardware y comprueba pines, coeficientes, unidades, identificación, clasificación, fallos de lectura, captura inválida sin contaminar el mapa, sensor ausente y parada antes de avanzar tras un giro con lectura inválida. También verifica los ciclos previos con fuente simulada y la compilación de la selección alternativa. No sustituye pruebas con SDK, sensores ni motores reales. En este entorno falta repetir la compilación ESP32 con PlatformIO y la comprobación física. Ningún ZIP se actualiza automáticamente.
# mousecode
