#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <Wire.h>

// ====== BIẾN NGOÀI (extern) ======
extern float gyroBiasX, gyroBiasY, gyroBiasZ;
constexpr float DEG2RAD = 3.14159265358979323846f / 180.0f;
// ====== HÀM ======
bool readIMU(float &ax, float &ay, float &az,
             float &gx, float &gy, float &gz);

void calibrateGyroBias(int samples = 500);

// ====== MADGWICK ======
class Madgwick {
public:

  Madgwick(float b = 0.08f);

  void update(float gx, float gy, float gz,
              float ax, float ay, float az, float dt);

  float getYaw();
  float getPitch();
  float getRoll();

  void rotateVectorByQuat(float vx, float vy, float vz,
                          float &ox, float &oy, float &oz);
private:
  float q0, q1, q2, q3;
  float beta;
};

// object global
extern Madgwick madgwick;

// task (FreeRTOS)
void taskIMU(void *pvParameters);

#endif