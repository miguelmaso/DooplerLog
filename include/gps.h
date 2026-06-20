#pragma once
#include <Arduino.h>

// ── Ajustes configurables ────────────────────────────────────────────
constexpr int   GPS_RX_PIN   = 16;
constexpr int   GPS_TX_PIN   = 17;
constexpr int   GPS_BAUD     = 38400;
constexpr int   GPS_RATE_HZ  = 1;      // frecuencia de medición: 1 o 5
constexpr int   AVERAGE_N    = 8;      // muestras para la media vectorial
constexpr float MIN_SPEED_KT = 0.15f;  // por debajo de esto no se muestra rumbo
constexpr int   CPU_FREQ_MHZ = 80;     // frecuencia de CPU: 80 o 240
// ────────────────────────────────────────────────────────────────────

void gpsInit();
void gpsPoll();
