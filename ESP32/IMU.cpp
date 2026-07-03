#include "IMU.h"
#include <math.h>
#define MPU_ADDR 0x68

bool readIMU(float &ax, float &ay, float &az,
             float &gx, float &gy, float &gz) {

  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B); // ACCEL_XOUT_H
  if (Wire.endTransmission(false) != 0) return false;
  Wire.requestFrom(MPU_ADDR, (uint8_t)14, (uint32_t)100);

  if (Wire.available() < 14) return false;

  int16_t rawAx = (Wire.read() << 8) | Wire.read();
  int16_t rawAy = (Wire.read() << 8) | Wire.read();
  int16_t rawAz = (Wire.read() << 8) | Wire.read();
  Wire.read(); Wire.read(); // bỏ qua nhiệt độ
  int16_t rawGx = (Wire.read() << 8) | Wire.read();
  int16_t rawGy = (Wire.read() << 8) | Wire.read();
  int16_t rawGz = (Wire.read() << 8) | Wire.read();

  // chuyển đổi sang đơn vị chuẩn
  const float A_SCALE = 9.80665f / 16384.0f; // ±2g
  const float G_SCALE = 1.0f / 131.0f;      // ±250°/s

  ax = rawAx * A_SCALE;
  ay = rawAy * A_SCALE;
  az = rawAz * A_SCALE;

  gx = rawGx * G_SCALE * DEG2RAD; // rad/s
  gy = rawGy * G_SCALE * DEG2RAD;
  gz = rawGz * G_SCALE * DEG2RAD;

  // trừ bias
  gx -= gyroBiasX;
  gy -= gyroBiasY;
  gz -= gyroBiasZ;

  return true;
}

void calibrateGyroBias(int samples) {
  Serial.println("=== Calibrating gyro... keep robot still ===");

  float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
  int valid = 0;

  for (int i = 0; i < samples; i++) {
    // Nếu bánh xe bắt đầu quay thì dừng ngay

    float ax, ay, az, gx, gy, gz;
    if (readIMU(ax, ay, az, gx, gy, gz)) {
      sumX += gx;
      sumY += gy;
      sumZ += gz;
      valid++;
    }

    delay(5); // giảm tải cảm biến, khoảng 100Hz
  }

  if (valid > 0) {
    gyroBiasX = sumX / (float)valid;
    gyroBiasY = sumY / (float)valid;
    gyroBiasZ = sumZ / (float)valid;
    Serial.println("Gyro bias calibrated:");
    Serial.print("  X: "); Serial.println(gyroBiasX);
    Serial.print("  Y: "); Serial.println(gyroBiasY);
    Serial.print("  Z: "); Serial.println(gyroBiasZ);
  } else {
    Serial.println("No valid IMU samples (calibration failed)."); 
  }}

Madgwick::Madgwick(float b) : beta(b) {
    q0 = 1.0f; q1 = q2 = q3 = 0.0f;
}
  // gx,gy,gz in rad/s; ax,ay,az in m/s^2 (or normalized); dt in seconds
void Madgwick::update(float gx, float gy, float gz,
                      float ax, float ay, float az, float dt) {
    if (dt <= 0.0f) return;

    // 1) normalize accelerometer (if 0, skip)
    float norm = sqrtf(ax*ax + ay*ay + az*az);
    if (norm == 0.0f) return;
    ax /= norm; ay /= norm; az /= norm;

    // short names
    float q0q0 = q0*q0;
    float q1q1 = q1*q1;
    float q2q2 = q2*q2;
    float q3q3 = q3*q3;

    // gradient descent algorithm corrective step
    // from Madgwick's original algorithm (expanded)
    float _2q0 = 2.0f * q0;
    float _2q1 = 2.0f * q1;
    float _2q2 = 2.0f * q2;
    float _2q3 = 2.0f * q3;
    float _4q0 = 4.0f * q0;
    float _4q1 = 4.0f * q1;
    float _4q2 = 4.0f * q2;
    float _8q1 = 8.0f * q1;
    float _8q2 = 8.0f * q2;
    float q0q1 = q0 * q1;
    float q0q2 = q0 * q2;
    float q0q3 = q0 * q3;
    float q1q2 = q1 * q2;
    float q1q3 = q1 * q3;
    float q2q3 = q2 * q3;

    // objective function and Jacobian (s0..s3)
    float s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
    float s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q1 - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
    float s2 = 4.0f * q0q0 * q2 + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
    float s3 = 4.0f * q1q1 * q3 - _2q1 * ax + 4.0f * q2q2 * q3 - _2q2 * ay;

    // NOTE: The above four lines are one form of compact expressions — but to avoid
    // possible transcription errors it's safer to use standard expanded form from Madgwick.
    // For clarity and correctness below we'll compute s0..s3 using the well-known expanded form:

    // Recompute s0..s3 using canonical expanded Madgwick formula:
    float f1 = 2.0f*(q1q3 - q0q2) - ax;
    float f2 = 2.0f*(q0q1 + q2q3) - ay;
    float f3 = 2.0f*(0.5f - q1q1 - q2q2) - az;

    // Jacobian transpose * f (gradient)
    s0 = -_2q2 * f1 + _2q1 * f2;
    s1 =  _2q3 * f1 + _2q0 * f2 - _4q1 * f3;
    s2 = -_2q0 * f1 + _2q3 * f2 - _4q2 * f3;
    s3 =  _2q1 * f1 + _2q2 * f2;

    // normalize step magnitude
    float sNorm = sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
    if (sNorm == 0.0f) return;
    s0 /= sNorm; s1 /= sNorm; s2 /= sNorm; s3 /= sNorm;

    // compute rate of change of quaternion from gyroscope
    float qDot0 = 0.5f * (-q1*gx - q2*gy - q3*gz) - beta * s0;
    float qDot1 = 0.5f * ( q0*gx + q2*gz - q3*gy) - beta * s1;
    float qDot2 = 0.5f * ( q0*gy - q1*gz + q3*gx) - beta * s2;
    float qDot3 = 0.5f * ( q0*gz + q1*gy - q2*gx) - beta * s3;

    // integrate
    q0 += qDot0 * dt;
    q1 += qDot1 * dt;
    q2 += qDot2 * dt;
    q3 += qDot3 * dt;

    // normalize quaternion
    float qNorm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
    if (qNorm == 0.0f) {
      // reset to identity to avoid NaNs
      q0 = 1.0f; q1 = q2 = q3 = 0.0f;
      return;
    }
    q0 /= qNorm; q1 /= qNorm; q2 /= qNorm; q3 /= qNorm;
  }

  // get yaw/pitch/roll (rad)
  float Madgwick::getYaw()   { return atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3)); }
  float Madgwick::getPitch() {
  float v = 2.0f*(q0*q2 - q3*q1);
  if (v > 1.0f) v = 1.0f;
  if (v < -1.0f) v = -1.0f;
  return asinf(v);
}
  float Madgwick::getRoll()  { return atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2)); }

  // rotate vector v by quaternion: v_out = q * v * q_conj
 void Madgwick::rotateVectorByQuat(float vx, float vy, float vz, float &ox, float &oy, float &oz) {
  float qx = q1, qy = q2, qz = q3;
  // t = 2 * cross(q_vec, v)
  float tx = 2.0f * (qy * vz - qz * vy);
  float ty = 2.0f * (qz * vx - qx * vz);
  float tz = 2.0f * (qx * vy - qy * vx);
  // cross(q_vec, t)
  float cx = qy * tz - qz * ty;
  float cy = qz * tx - qx * tz;
  float cz = qx * ty - qy * tx;
  ox = vx + q0 * tx + cx;
  oy = vy + q0 * ty + cy;
  oz = vz + q0 * tz + cz;
}

void taskIMU(void *pvParameters) {
  unsigned long last = millis();
  while (1) {
    float ax, ay, az, gx, gy, gz;
    readIMU(ax, ay, az, gx, gy, gz);  

    unsigned long now = millis();
    float dt = (now - last) / 1000.0f;
    last = now;

    madgwick.update(gx, gy, gz, ax, ay, az, dt);

    vTaskDelay(10 / portTICK_PERIOD_MS); // cập nhật mỗi 10ms (100Hz)
  }
}
float gyroBiasX = 0;
float gyroBiasY = 0;
float gyroBiasZ = 0;

Madgwick madgwick(0.08f);