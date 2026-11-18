#include <SimpleFOC.h>

// FOC_ESP32_V1.1 P2: CS=GPIO13; default ESP32 SPI pins are SCK=18, MISO=19, MOSI=23.
constexpr int SENSOR_CS = 13;
constexpr float TWO_PI_F = 6.28318530718f;
constexpr uint16_t SENSOR_CPR = 16384;

MagneticSensorSPI sensor = MagneticSensorSPI(AS5047_SPI, SENSOR_CS);

void setup() {
  Serial.begin(115200);
  sensor.init();
  Serial.println("AS5047P encoder test: rotate the magnet by hand.");
  Serial.println("count (0-16383), angle_deg");
}

void loop() {
  float angle_rad = sensor.getSensorAngle();
  uint16_t count = (uint16_t)(angle_rad * SENSOR_CPR / TWO_PI_F);
  float angle_deg = angle_rad * 360.0f / TWO_PI_F;

  Serial.print(count);
  Serial.print(", ");
  Serial.println(angle_deg, 2);
  delay(100);
}
