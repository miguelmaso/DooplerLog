#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>
#include <math.h>

SFE_UBLOX_GNSS gps;

// --- Moving average vectorial ---
constexpr int N = 10;
constexpr float MMS_TO_KT  = 1.0f / 514.444f;   // mm/s to knots
constexpr float MIN_SPEED_KT = 0.15f;           // noise threshold

float vn_buf[N] = {0};  // North [kt]
float ve_buf[N] = {0};  // East  [kt]
int   buf_idx   = 0;

void setup() {
  Serial.begin(115200);
  Wire.begin(21, 22);  // SDA, SCL

  if (!gps.begin()) {
    Serial.println("GPS not found on I2C — check connections");
    while (1);
  }

  gps.setI2COutput(COM_TYPE_UBX);  // Solo protocolo UBX, sin NMEA
  gps.setNavigationFrequency(5);   // 5 Hz — más muestras para el promedio
  gps.setAutoPVT(true);            // Push automático, sin polling
  gps.saveConfiguration();

  Serial.println("GPS initialized — waiting fix...");
}

void loop() {
  if (!gps.getPVT()) return;

  uint8_t fixType = gps.getFixType();
  uint8_t numSV   = gps.getSIV();

  if (fixType < 2) {
    Serial.print("Sin fix | Satélites: "); Serial.println(numSV);
    return;
  }

  // Velocidad vectorial cruda en mm/s (int32 — alta resolución)
  int32_t velN_raw = gps.getNedNorthVel();   // Norte  mm/s
  int32_t velE_raw = gps.getNedEastVel();   // Este   mm/s
  uint32_t sAcc   = gps.getSpeedAccEst(); // Estimación de error (mm/s)

  // Convertir a nudos manteniendo signo
  float vn = velN_raw * MMS_TO_KT;
  float ve = velE_raw * MMS_TO_KT;

  // Buffer circular
  vn_buf[buf_idx] = vn;
  ve_buf[buf_idx] = ve;
  buf_idx = (buf_idx + 1) % N;

  // Vector promedio
  float vn_avg = 0, ve_avg = 0;
  for (int i = 0; i < N; i++) {
    vn_avg += vn_buf[i];
    ve_avg += ve_buf[i];
  }
  vn_avg /= N;
  ve_avg /= N;

  float speed_raw = sqrt(vn * vn + ve * ve);
  float speed_avg = sqrt(vn_avg * vn_avg + ve_avg * ve_avg);

  float hdg_raw = atan2(ve, vn)         * RAD_TO_DEG; if (hdg_raw < 0) hdg_raw += 360;
  float hdg_avg = atan2(ve_avg, vn_avg) * RAD_TO_DEG; if (hdg_avg < 0) hdg_avg += 360;

  // Output
  Serial.print("Sat:"); Serial.print(numSV);
  Serial.print(" | sAcc:"); Serial.print(sAcc); Serial.print("mm/s");
  Serial.print(" | VN:"); Serial.print(velN_raw); Serial.print(" VE:"); Serial.print(velE_raw); Serial.print(" mm/s");
  Serial.print(" | Vel raw:"); Serial.print(speed_raw, 3);
  Serial.print(" avg:"); Serial.print(speed_avg, 3); Serial.print(" kt");
  if (speed_avg > MIN_SPEED_KT) {
    Serial.print(" | Hdg raw:"); Serial.print(hdg_raw, 1);
    Serial.print(" avg:"); Serial.print(hdg_avg, 1); Serial.print("°");
  } else {
    Serial.print(" | Hdg: --- (parado)");
  }
  Serial.println();
}