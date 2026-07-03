#ifndef Lidar_VFH_H
#define Lidar_VFH_H
#include <Arduino.h>
#include <Wire.h>
#define RXD2 34
#define TXD2 -1
// ===== VFH Parameters =====
#define NUM_SECTORS 72                     // 360 độ chia cho 5 độ = 72 sectors
// ===== VFH+ 2D OCCUPANCY GRID (40x40) =====
#define GRID_SIZE 30        
#define CELL_SIZE 33.0f     // mm (Kích thước 1 ô, có thể đổi tùy ý)
extern float polar_histogram[NUM_SECTORS]; 
extern float smoothed_histogram[NUM_SECTORS];
extern float occupancy[GRID_SIZE][GRID_SIZE]; // Lưới xác suất (0.0 -> 1.0)
extern float lidar_dist[360];
extern uint16_t lidar_intensity[360];
extern float grid_origin_x ; 
extern float grid_origin_y ;
extern float prevBestAngle;
extern float x;
extern float y;
extern float theta;
 // Biểu đồ chướng ngại vật
constexpr float MAX_OBS_DIST = 400.0;         // mm - Khoảng cách tối đa quan tâm (1 mét)
constexpr float VFH_THRESHOLD = 2.5;           // Ngưỡng mật độ - trên mừc này coi là có vật cản
constexpr float DECAY_RATE = 0.4;             // Hệ số làm mờ bản đồ cũ (0.0 -> 1.0)
// ===== Robot Parameters =====
constexpr float TPR = 616*0.25/(2*PI*0.072);
constexpr float ROBOT_RADIUS = 130.0; // Bán kính xe (mm) - Ví dụ xe rộng 30cm -> Bán kính 15cm
constexpr float SAFE_MARGIN = 15.0;   // Khoảng cách an toàn muốn cách xa thêm (mm)

struct LidarPoint {
  float angle;       // Góc (độ)
  uint16_t distance; // Khoảng cách (mm)
  uint8_t intensity; // Cường độ tín hiệu
};
extern LidarPoint currentPoints[12];
void shiftGrid(int shift_x, int shift_y);
void updateRayBresenham(int x0, int y0, int x1, int y1);
void beginLidar();
void taskLidar(void *pvParameters);
void smoothHistogram();
float computeVFH(float targetAngle, float currentTheta);
#endif