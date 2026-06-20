#include <Arduino.h>
#include <WiFi.h>
#include "gps.h"
#include "display.h"

void setup() {
  WiFi.mode(WIFI_OFF);
  btStop();
  setCpuFrequencyMhz(CPU_FREQ_MHZ);

  Serial.begin(115200);
  delay(500);

  displayInit();
  gpsInit();
}

void loop() {
  gpsPoll();

  if (gpsData.updated) {
    gpsData.updated = false;
    displayUpdate(gpsData);
  }
}
