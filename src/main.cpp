#include <Arduino.h>
#include <WiFi.h>
#include "gps.h"

void setup() {
  WiFi.mode(WIFI_OFF);
  btStop();
  setCpuFrequencyMhz(CPU_FREQ_MHZ);

  Serial.begin(115200);
  delay(500);

  gpsInit();
}

void loop() {
  gpsPoll();
}
