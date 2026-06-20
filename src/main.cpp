#include <Arduino.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

TinyGPSPlus gps;
HardwareSerial GPSSerial(2); // UART2

// Config Moving Average Mean
const int N = 4;           // Ventana (muestras). Ajusta según respuesta deseada
float vx_buf[N] = {0};     // Componente Este  (speed * sin(heading))
float vy_buf[N] = {0};     // Componente Norte (speed * cos(heading))
int buf_idx = 0;

// Below this speed, the velocity is not reliable
const float MIN_SPEED_KT = 0.9;

void setup() {
  Serial.begin(115200);
  GPSSerial.begin(38400, SERIAL_8N1, 16, 17); // RX=16, TX=17
  Serial.println("Esperando señal GPS...");
}

void loop() {
  while (GPSSerial.available() > 0) {
    gps.encode(GPSSerial.read());
  }

  static unsigned long last = 0;
  if (millis() - last > 1000) {
    last = millis();

    if (!gps.location.isValid()) {
      Serial.print("Satélites: "); Serial.print(gps.satellites.value());
      Serial.println(" | No fix...");
      return;
    }

    float speed_kt  = gps.speed.knots();
    float course_deg = gps.course.isValid() ? gps.course.deg() : 0.0;
    float course_rad = course_deg * DEG_TO_RAD;

    // Decompose cartesian speed (North = 0 DEG)
    float vx = speed_kt * sin(course_rad);
    float vy = speed_kt * cos(course_rad);
    // float vx = (speed_kt > MIN_SPEED_KT) ? speed_kt * sin(course_rad) : 0.0;
    // float vy = (speed_kt > MIN_SPEED_KT) ? speed_kt * cos(course_rad) : 0.0;

    // Insert in cyclic buffer
    vx_buf[buf_idx] = vx;
    vy_buf[buf_idx] = vy;
    buf_idx = (buf_idx + 1) % N;
    
    // Compute average speed
    float vx_avg = 0, vy_avg = 0;
    for (int i = 0; i < N; i++) {
      vx_avg += vx_buf[i];
      vy_avg += vy_buf[i];
    }
    vx_avg /= N;
    vy_avg /= N;

    // Speed (SOG) and heading
    float speed_smooth  = sqrt(vx_avg * vx_avg + vy_avg * vy_avg);
    float heading_smooth = atan2(vx_avg, vy_avg) * RAD_TO_DEG;
    if (heading_smooth < 0) heading_smooth += 360.0;

    // Output
    Serial.print("Sat: ");      Serial.print(gps.satellites.value());
    Serial.print(" | Vel raw: "); Serial.print(speed_kt, 2);     Serial.print(" kt");
    Serial.print(" | Vel avg: "); Serial.print(speed_smooth, 2); Serial.print(" kt");

    if (speed_smooth > MIN_SPEED_KT) {
      Serial.print(" | Rumbo raw: "); Serial.print(course_deg, 1);    Serial.print("°");
      Serial.print(" | Rumbo avg: "); Serial.print(heading_smooth, 1); Serial.print("°");
    } else {
      Serial.print(" | Rumbo: --- (parado)");
    }
    Serial.println();
  }
}
