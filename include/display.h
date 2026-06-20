#pragma once
#include "gps.h"

// ── Ajustes display ──────────────────────────────────────────────────
constexpr int DISP_SCLK = 18;
constexpr int DISP_MOSI = 23;
constexpr int DISP_CS   = 5;
// ────────────────────────────────────────────────────────────────────

void displayInit();
void displayUpdate(const GpsData& d);
