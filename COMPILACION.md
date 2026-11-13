# Compilación comprobada en PlatformIO

- Fecha: 2026-11-05.
- Proyecto: `unificacacion/platformio.ini`, entorno `esp32dev`.
- PlatformIO Core: 6.2.0.
- Espressif32: 6.9.0.
- Framework: Arduino ESP32 2.0.17 (`3.20017.241212+sha.dcc1105b`).
- Compilador: xtensa-esp32 GCC 8.4.0, 2021r2-patch5.
- ESP32Encoder: 0.12.0.
- Resultado: **SUCCESS**, compilación y enlace reales verificados tras actualizar la etapa de control de bajo nivel.
- RAM estática: 50.692 bytes de 327.680 (15,5 %).
- Programa: 319.653 bytes de 1.310.720 (24,4 %).
- Archivo `.pio/build/esp32dev/firmware.bin` generado exitosamente.

## Actualizaciones de Control de Bajo Nivel
- `RPM_MAX`: Ajustado a 250.0 RPM (con recálculo de `W_TOPE` y límites del lazo/mezclador).
- Nuevo criterio de asentamiento / cambio de referencia:
  - `TOL_S_FIN = 0.0015 m`
  - `TOL_PHI_FIN = 1.0°`
  - `VEL_ASENTADO_RPM = 10.0 RPM`
  - Tiempos de asentado: `T_ESPERA_MIN_MS = 200 ms`, `T_ASENTADO_MS = 150 ms`, `T_ASENTAR_MAX_MS = 3000 ms`.
  - Notificaciones en monitor serie al asentar o agotar tiempo con errores de distancia y ángulo.
- Inicialización y calibración del MPU6050 y encoders idéntica a la nueva etapa de pruebas.
