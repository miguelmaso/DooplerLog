#include "gps.h"
#include <math.h>

GpsData gpsData;

static HardwareSerial GPSSerial(2);

static constexpr float MMS_TO_KT = 1.0f / 514.444f;

static float vn_buf[AVERAGE_N] = {};
static float ve_buf[AVERAGE_N] = {};
static int   buf_idx = 0;

// ── UBX parser ───────────────────────────────────────────────────────
enum { S_SYNC1, S_SYNC2, S_CLASS, S_ID, S_LEN_L, S_LEN_H, S_PAYLOAD, S_CK_A, S_CK_B };
static uint8_t  ubxState = S_SYNC1;
static uint8_t  ubxClass, ubxId;
static uint16_t ubxLen, ubxIdx;
static uint8_t  ubxBuf[256];
static uint8_t  ckA, ckB, rxCkA;

static void ubxChecksum(const uint8_t* data, size_t len, uint8_t& a, uint8_t& b) {
  a = b = 0;
  for (size_t i = 0; i < len; i++) { a += data[i]; b += a; }
}

static void sendUBX(const uint8_t* pkt, size_t len) {
  GPSSerial.write(pkt, len);
  delay(150);
  while (GPSSerial.available()) GPSSerial.read();
}

// Construye y envía un CFG-VALSET con una sola clave/valor
static void sendCfgValset(uint32_t key, const uint8_t* val, uint8_t valLen) {
  uint16_t payloadLen = 4 + 4 + valLen;
  uint8_t  pkt[32];
  pkt[0] = 0xB5; pkt[1] = 0x62;
  pkt[2] = 0x06; pkt[3] = 0x8A;
  pkt[4] = payloadLen & 0xFF; pkt[5] = (payloadLen >> 8) & 0xFF;
  pkt[6] = 0x00; pkt[7] = 0x01;
  pkt[8] = 0x00; pkt[9] = 0x00;
  pkt[10] = key & 0xFF; pkt[11] = (key >> 8) & 0xFF;
  pkt[12] = (key >> 16) & 0xFF; pkt[13] = (key >> 24) & 0xFF;
  for (uint8_t i = 0; i < valLen; i++) pkt[14 + i] = val[i];
  uint8_t a, b;
  ubxChecksum(pkt + 2, 4 + payloadLen, a, b);
  pkt[14 + valLen] = a;
  pkt[15 + valLen] = b;
  sendUBX(pkt, 16 + valLen);
}

static void configureGPS() {
  uint8_t off = 0x00, on = 0x01;
  sendCfgValset(0x10740003, &off, 1);  // CFG-UART1OUTPROT-NMEA = 0
  sendCfgValset(0x10740002, &on,  1);  // CFG-UART1OUTPROT-UBX  = 1
  sendCfgValset(0x20910007, &on,  1);  // CFG-MSGOUT-UBX_NAV_PVT_UART1 = 1

  uint16_t period_ms = 1000 / GPS_RATE_HZ;
  uint8_t  rate[2]   = { (uint8_t)(period_ms & 0xFF), (uint8_t)(period_ms >> 8) };
  sendCfgValset(0x30210001, rate, 2);  // CFG-RATE-MEAS
}

// ── Procesado NAV-PVT ────────────────────────────────────────────────
static void processNavPVT(const uint8_t* buf) {
  uint8_t fixType = buf[20];
  uint8_t numSV   = buf[23];

  gpsData.sats    = numSV;
  gpsData.hasFix  = (fixType >= 2);
  gpsData.updated = true;

  if (!gpsData.hasFix) {
    Serial.print("Sin fix | Sat: "); Serial.println(numSV);
    return;
  }

  int32_t  velN_raw, velE_raw;
  uint32_t sAcc;
  memcpy(&velN_raw, buf + 48, 4);
  memcpy(&velE_raw, buf + 52, 4);
  memcpy(&sAcc,     buf + 68, 4);

  float vn = velN_raw * MMS_TO_KT;
  float ve = velE_raw * MMS_TO_KT;

  vn_buf[buf_idx] = vn;
  ve_buf[buf_idx] = ve;
  buf_idx = (buf_idx + 1) % AVERAGE_N;

  float vn_avg = 0, ve_avg = 0;
  for (int i = 0; i < AVERAGE_N; i++) { vn_avg += vn_buf[i]; ve_avg += ve_buf[i]; }
  vn_avg /= AVERAGE_N;
  ve_avg /= AVERAGE_N;

  float speed_raw = sqrtf(vn * vn + ve * ve);
  float speed_avg = sqrtf(vn_avg * vn_avg + ve_avg * ve_avg);
  float hdg_raw   = atan2f(ve, vn)         * RAD_TO_DEG; if (hdg_raw < 0) hdg_raw += 360;
  float hdg_avg   = atan2f(ve_avg, vn_avg) * RAD_TO_DEG; if (hdg_avg < 0) hdg_avg += 360;

  gpsData.speed_kt = speed_avg;
  gpsData.hdg_deg  = hdg_avg;
  gpsData.sAcc_mms = sAcc;

  Serial.print("Sat:"); Serial.print(numSV);
  Serial.print(" | sAcc:"); Serial.print(sAcc); Serial.print("mm/s");
  Serial.print(" | Vel raw:"); Serial.print(speed_raw, 3);
  Serial.print(" avg:"); Serial.print(speed_avg, 3); Serial.print(" kt");
  if (speed_avg > MIN_SPEED_KT) {
    Serial.print(" | Hdg raw:"); Serial.print(hdg_raw, 1);
    Serial.print(" avg:"); Serial.print(hdg_avg, 1); Serial.print("deg");
  } else {
    Serial.print(" | Hdg: --- (parado)");
  }
  Serial.println();
}

static void parseUBX(uint8_t b) {
  switch (ubxState) {
    case S_SYNC1:   ubxState = (b == 0xB5) ? S_SYNC2 : S_SYNC1; break;
    case S_SYNC2:   ubxState = (b == 0x62) ? S_CLASS  : S_SYNC1; break;
    case S_CLASS:   ubxClass = b; ckA = b; ckB = b; ubxState = S_ID; break;
    case S_ID:      ubxId = b; ckA += b; ckB += ckA; ubxState = S_LEN_L; break;
    case S_LEN_L:   ubxLen = b; ckA += b; ckB += ckA; ubxState = S_LEN_H; break;
    case S_LEN_H:
      ubxLen |= (uint16_t)b << 8; ckA += b; ckB += ckA;
      ubxIdx = 0;
      ubxState = (ubxLen == 0) ? S_CK_A : S_PAYLOAD;
      break;
    case S_PAYLOAD:
      if (ubxIdx < sizeof(ubxBuf)) ubxBuf[ubxIdx] = b;
      ckA += b; ckB += ckA;
      if (++ubxIdx >= ubxLen) ubxState = S_CK_A;
      break;
    case S_CK_A:    rxCkA = b; ubxState = S_CK_B; break;
    case S_CK_B:
      if (rxCkA == ckA && b == ckB && ubxClass == 0x01 && ubxId == 0x07 && ubxLen >= 92)
        processNavPVT(ubxBuf);
      ubxState = S_SYNC1;
      break;
  }
}

// ── API pública ──────────────────────────────────────────────────────
void gpsInit() {
  GPSSerial.setRxBufferSize(2048);
  GPSSerial.begin(GPS_BAUD, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  delay(1000);
  Serial.println("Configurando GPS...");
  configureGPS();
  Serial.println("GPS configurado — esperando fix...");
}

void gpsPoll() {
  while (GPSSerial.available())
    parseUBX(GPSSerial.read());
}
