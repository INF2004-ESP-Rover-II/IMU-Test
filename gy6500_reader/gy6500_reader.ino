/*
 * GY-6500 (MPU6500) reader for the Sensor Pico
 *
 * Reads accelerometer + gyroscope over I2C and computes a running
 * pitch/roll estimate via a complementary filter.
 *
 * IMPORTANT: the MPU6500 is a 6-axis part (accel + gyro only)
 * Practical consequence: pitch/roll (tilt) stay accurate long-term,
 * but yaw (heading) will slowly drift, since it comes only from
 * integrating the gyro over time with nothing to correct it against.
 * ArUco handles periodic absolute correction.
 *
 * Wiring (one sensor):
 *   VCC -> 3V3   (NOT 5V -- the MPU6500 die is 3.3V-logic only)
 *   GND -> GND
 *   SCL -> Sensor Pico GPIO7 (I2C1 SCL)
 *   SDA -> Sensor Pico GPIO6 (I2C1 SDA)
 *   AD0 -> GND   (this sensor answers at I2C address 0x68)
 *
 * For a second sensor on the same bus:
 *   AD0 -> 3V3   (that sensor answers at 0x69 instead, no conflict)
 *   everything else wired identically, in parallel on the same bus
 */

#include <Wire.h>

// MPU6500 register map (only the registers this sketch needs)
#define MPU6500_ADDR_LOW   0x68  // AD0 tied to GND
#define MPU6500_ADDR_HIGH  0x69  // AD0 tied to 3V3
#define REG_WHO_AM_I       0x75
#define REG_PWR_MGMT_1     0x6B
#define REG_GYRO_CONFIG    0x1B
#define REG_ACCEL_CONFIG   0x1C
#define REG_ACCEL_XOUT_H   0x3B
#define WHO_AM_I_EXPECTED  0x70  // MPU6500's own device ID

// I2C1 pins on the RP2040 -- (arduino-pico's defaults)
// change if board wiring differs
#define I2C_SDA_PIN 6
#define I2C_SCL_PIN 7

struct ImuReading {
  float accel_g[3];   // x, y, z in g
  float gyro_dps[3];  // x, y, z in degrees/sec
};

bool mpu6500_init(uint8_t addr) {
  // Confirm talking to an MPU6500 before configuring
  Wire1.beginTransmission(addr);
  Wire1.write(REG_WHO_AM_I);
  Wire1.endTransmission(false);
  Wire1.requestFrom(addr, (uint8_t)1);
  if (Wire1.available() != 1) return false;
  uint8_t who = Wire1.read();
  // Error handling
  if (who != WHO_AM_I_EXPECTED) {
    Serial.printf("WHO_AM_I mismatch at 0x%02X: got 0x%02X, expected 0x%02X\n",
                  addr, who, WHO_AM_I_EXPECTED);
    return false;
  }

  // Wake the chip -- boots in sleep mode by default
  Wire1.beginTransmission(addr);
  Wire1.write(REG_PWR_MGMT_1);
  Wire1.write(0x00);
  Wire1.endTransmission();
  delay(50);

  // Gyro range: +/-500 dps -- widen to 1000/2000 if the
  // readings clip/saturate during sharp turns)
  Wire1.beginTransmission(addr);
  Wire1.write(REG_GYRO_CONFIG);
  Wire1.write(0x08);  // FS_SEL = 1 -> +/-500 dps
  Wire1.endTransmission();

  // Accel range: +/-4g
  Wire1.beginTransmission(addr);
  Wire1.write(REG_ACCEL_CONFIG);
  Wire1.write(0x08);  // AFS_SEL = 1 -> +/-4g
  Wire1.endTransmission();

  return true;
}

bool mpu6500_read(uint8_t addr, ImuReading &out) {
  Wire1.beginTransmission(addr);
  Wire1.write(REG_ACCEL_XOUT_H);
  Wire1.endTransmission(false);
  Wire1.requestFrom(addr, (uint8_t)14);  // accel(6) + temp(2) + gyro(6)
  if (Wire1.available() != 14) return false;

  int16_t raw[7];
  for (int i = 0; i < 7; i++) {
    raw[i] = (Wire1.read() << 8) | Wire1.read();
  }

  const float accel_scale = 4.0f / 32768.0f;    // matches +/-4g config above
  const float gyro_scale  = 500.0f / 32768.0f;  // matches +/-500 dps config above

  out.accel_g[0] = raw[0] * accel_scale;
  out.accel_g[1] = raw[1] * accel_scale;
  out.accel_g[2] = raw[2] * accel_scale;
  // raw[3] is the onboard temperature sensor, unused here
  out.gyro_dps[0] = raw[4] * gyro_scale;
  out.gyro_dps[1] = raw[5] * gyro_scale;
  out.gyro_dps[2] = raw[6] * gyro_scale;

  return true;
}

// Complementary filter state (persists between loop() calls)
float pitch = 0.0f, roll = 0.0f;
unsigned long last_update_us = 0;

void update_tilt(const ImuReading &r) {
  unsigned long now = micros();
  float dt = (last_update_us == 0) ? 0.01f : (now - last_update_us) / 1e6f;
  last_update_us = now;

  // Tilt angle computed from gravity direction alone -- noisy on a
  // per-sample basis, but does not drift over time
  float accel_pitch = atan2(-r.accel_g[0],
                             sqrt(r.accel_g[1] * r.accel_g[1] + r.accel_g[2] * r.accel_g[2]))
                       * 180.0f / PI;
  float accel_roll = atan2(r.accel_g[1], r.accel_g[2]) * 180.0f / PI;

  // Blend: trust the gyro for smooth short-term change, pull gently
  // back toward the accelerometer's long-term estimate so the two
  // sensors' weaknesses cancel out (gyro drift vs accel noise)
  const float alpha = 0.98f;
  pitch = alpha * (pitch + r.gyro_dps[1] * dt) + (1 - alpha) * accel_pitch;
  roll  = alpha * (roll  + r.gyro_dps[0] * dt) + (1 - alpha) * accel_roll;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  Wire1.setSDA(I2C_SDA_PIN);
  Wire1.setSCL(I2C_SCL_PIN);
  Wire1.begin();
  Wire1.setClock(400000);  // 400kHz fast mode -- fine for short wire runs

  if (!mpu6500_init(MPU6500_ADDR_LOW)) {
    Serial.println("Failed to init MPU6500 at 0x68 -- check wiring/address");
    while (1) delay(1000);
  }
  Serial.println("MPU6500 ready");
}

void loop() {
  ImuReading r;
  if (mpu6500_read(MPU6500_ADDR_LOW, r)) {
    update_tilt(r);
    Serial.printf("pitch=%.2f  roll=%.2f  |  gyroZ=%.2f dps (raw yaw rate)\n",
                  pitch, roll, r.gyro_dps[2]);
    // TODO: feed gyroZ (yaw rate) into your rover's heading estimate,
    // and forward pitch/roll/heading to Robo Pico or the laptop over
    // whichever link you're already using for sensor data.
  }
  delay(10);  // roughly 100Hz loop
}
