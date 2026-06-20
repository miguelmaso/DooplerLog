#include "display.h"
#include <Adafruit_SharpMem.h>

static Adafruit_SharpMem display(DISP_SCLK, DISP_MOSI, DISP_CS, 128, 128);

#define BLACK 0
#define WHITE 1

void displayInit() {
  display.begin();
  display.clearDisplay();
  display.refresh();
}

void displayUpdate(const GpsData& d) {
  display.clearDisplay();
  display.setTextColor(BLACK);

  if (!d.hasFix) {
    display.setTextSize(2);
    display.setCursor(22, 28);
    display.print("Sin fix");
    display.setTextSize(1);
    display.setCursor(46, 64);
    display.print("SAT: "); display.print(d.sats);
    display.refresh();
    return;
  }

  char buf[16];

  // Velocidad — número grande
  display.setTextSize(3);
  snprintf(buf, sizeof(buf), "%5.2f", d.speed_kt);
  display.setCursor(4, 6);
  display.print(buf);
  display.setTextSize(2);
  display.setCursor(4, 38);
  display.print("kt");

  // Línea separadora
  display.drawFastHLine(0, 60, 128, BLACK);

  // Rumbo
  display.setTextSize(2);
  display.setCursor(4, 66);
  if (d.speed_kt > MIN_SPEED_KT) {
    snprintf(buf, sizeof(buf), "HDG %05.1f", d.hdg_deg);
    display.print(buf);
  } else {
    display.print("HDG   ---");
  }

  // Línea separadora
  display.drawFastHLine(0, 90, 128, BLACK);

  // Calidad de señal
  display.setTextSize(1);
  display.setCursor(4, 96);
  display.print("SAT: "); display.print(d.sats);
  display.setCursor(4, 108);
  display.print("sAcc: "); display.print(d.sAcc_mms); display.print(" mm/s");

  display.refresh();
}
