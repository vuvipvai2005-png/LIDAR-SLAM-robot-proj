#include "Lidar_VFH.h"
#include <math.h>

float x=0, y=0, theta=0;
float polar_histogram[NUM_SECTORS] = {0};
float smoothed_histogram[NUM_SECTORS] = {0};
float occupancy[GRID_SIZE][GRID_SIZE] = {0.0f};
float lidar_dist[360] = {10.0};
uint16_t lidar_intensity[360] = {0};
float grid_origin_x = - (GRID_SIZE / 2.0f) * CELL_SIZE;
float grid_origin_y = - (GRID_SIZE / 2.0f) * CELL_SIZE;
float prevBestAngle = 0;
LidarPoint currentPoints[12] = {};
void shiftGrid(int shift_x, int shift_y) {
  float temp[GRID_SIZE][GRID_SIZE] = {0.0f};
  for (int i = 0; i < GRID_SIZE; i++) {
    for (int j = 0; j < GRID_SIZE; j++) {
      int old_i = i + shift_x;
      int old_j = j + shift_y;
      if (old_i >= 0 && old_i < GRID_SIZE && old_j >= 0 && old_j < GRID_SIZE) {
        temp[i][j] = occupancy[old_i][old_j];
      }
    }
  }
  memcpy(occupancy, temp, sizeof(occupancy));
}
// Thuật toán Bresenham: Quét từ mũi xe (x0, y0) đến vật cản (x1, y1)
void updateRayBresenham(int x0, int y0, int x1, int y1) {
  int dx = abs(x1 - x0), sx = x0 < x1 ? 1 : -1;
  int dy = -abs(y1 - y0), sy = y0 < y1 ? 1 : -1;
  int err = dx + dy, e2;

  while (true) {
    if (x0 == x1 && y0 == y1) break; // Chạm đến ô chứa vật cản thì dừng
    
    // Giảm xác suất (Certainty) của các ô tia Lidar bay xuyên qua
    if (x0 >= 0 && x0 < GRID_SIZE && y0 >= 0 && y0 < GRID_SIZE) {
      occupancy[x0][y0] -= 0.05f; 
      if (occupancy[x0][y0] < 0.0f) occupancy[x0][y0] = 0.0f;
    }

    e2 = 2 * err;
    if (e2 >= dy) { err += dy; x0 += sx; }
    if (e2 <= dx) { err += dx; y0 += sy; }
  }
}
// Hàm khởi tạo Serial cho LiDAR
void beginLidar() {
  Serial2.setRxBufferSize(1024);
  Serial2.begin(115200, SERIAL_8N1, RXD2, TXD2);
}
void taskLidar(void *pvParameters) {
  static const int PACKET_SIZE = 47;
  static uint8_t buffer[47];
  static int bufferIndex = 0;
  static unsigned long lastUpdate = 0;
  while (1) {
    float cur_x_mm = x * 1000.0f;
    float cur_y_mm = y * 1000.0f;
    float cur_theta = theta;
    if (millis() - lastUpdate > 30) {
      float expected_origin_x = cur_x_mm - (GRID_SIZE / 2.0f) * CELL_SIZE;
      float expected_origin_y = cur_y_mm - (GRID_SIZE / 2.0f) * CELL_SIZE;
      int shift_x = round((grid_origin_x - expected_origin_x) / CELL_SIZE);
      int shift_y = round((grid_origin_y - expected_origin_y) / CELL_SIZE);
      if (shift_x != 0 || shift_y != 0) {
        shiftGrid(-shift_x, -shift_y);
        grid_origin_x -= shift_x * CELL_SIZE;
        grid_origin_y -= shift_y * CELL_SIZE;
      }
      for (int i = 0; i < NUM_SECTORS; i++) polar_histogram[i] = 0.0f;
      for (int i = 0; i < GRID_SIZE; i++) {
        for (int j = 0; j < GRID_SIZE; j++) {
          occupancy[i][j] *= 0.90f;
          if (occupancy[i][j] < 0.1f) {
              occupancy[i][j] = 0.0f;
              continue;
          }
          if (occupancy[i][j] > 0.4f) {
            float cell_x = grid_origin_x + (i + 0.5f) * CELL_SIZE;
            float cell_y = grid_origin_y + (j + 0.5f) * CELL_SIZE;
            float dx = cell_x - cur_x_mm;
            float dy = cell_y - cur_y_mm;
            float dist = sqrt(dx * dx + dy * dy);
            if (dist < ROBOT_RADIUS) continue;
            if (dist < MAX_OBS_DIST) {
              float angle_global = atan2(dy, dx);
              float rel_ang = angle_global - cur_theta;
              while (rel_ang < 0) rel_ang += 2 * PI;
              while (rel_ang >= 2 * PI) rel_ang -= 2 * PI;
              int sector = (int)((rel_ang * 180.0f / PI) / 5.0f) % NUM_SECTORS;
              float c_sq = occupancy[i][j] * occupancy[i][j];
              float magnitude = c_sq * 10.0f * (1.0f - (dist / MAX_OBS_DIST));
              float effective_radius = ROBOT_RADIUS + SAFE_MARGIN;
              float ratio = constrain(effective_radius / dist, -1.0f, 1.0f);
              float enlarge_angle_rad = asinf(ratio);
              int enlarge_sectors = ceil((enlarge_angle_rad * 180.0f / PI) / 5.0f);
              for (int s = -enlarge_sectors; s <= enlarge_sectors; s++) {
                int target = (sector + s + NUM_SECTORS) % NUM_SECTORS;
                polar_histogram[target] += magnitude;
              }
            }
          }
        }
      }
      lastUpdate = millis();
    }
    while (Serial2.available()) {
      uint8_t b = Serial2.read();
      if (bufferIndex == 0 && b != 0x54) continue;
      if (bufferIndex == 1 && b != 0x2C) { bufferIndex = 0; continue; }
      buffer[bufferIndex++] = b;
      if (bufferIndex == PACKET_SIZE) {
        bufferIndex = 0;
        float start_angle = ((buffer[5] << 8) | buffer[4]) / 100.0f;
        float end_angle   = ((buffer[43] << 8) | buffer[42]) / 100.0f;
        float step = (end_angle >= start_angle) ? (end_angle - start_angle) / 11.0f : (end_angle + 360.0f - start_angle) / 11.0f;
        for (int i = 0; i < 12; i++) {
          int offset = 6 + (i * 3);
          uint16_t dist = (buffer[offset + 1] << 8) | buffer[offset];
          uint8_t intensity = buffer[offset + 2];
          float raw_angle = start_angle + step * i;
          float angle_rel_deg = 360.0f - raw_angle + 180;
          while (angle_rel_deg >= 360.0f) angle_rel_deg -= 360.0f;
          while (angle_rel_deg < 0.0f) angle_rel_deg += 360.0f;
          int deg_index = (int)angle_rel_deg % 360;
          if (dist > 0 && dist < 8000) {
            if (intensity < 20){
                lidar_dist[deg_index] = 10.0f;
                lidar_intensity[deg_index] = 0;
                continue;
            }
            lidar_dist[deg_index] = dist / 1000.0f;
            lidar_intensity[deg_index] = intensity;
            float angle_rel_rad = angle_rel_deg * PI / 180.0f;
            float angle_global_rad = cur_theta + angle_rel_rad;
            float hit_x = cur_x_mm + dist * cos(angle_global_rad);
            float hit_y = cur_y_mm + dist * sin(angle_global_rad);
            int gx = floor((hit_x - grid_origin_x) / CELL_SIZE);
            int gy = floor((hit_y - grid_origin_y) / CELL_SIZE);
            int cx = floor((cur_x_mm - grid_origin_x) / CELL_SIZE);
            int cy = floor((cur_y_mm - grid_origin_y) / CELL_SIZE);
            updateRayBresenham(cx, cy, gx, gy);
            if (gx >= 0 && gx < GRID_SIZE && gy >= 0 && gy < GRID_SIZE) {
              occupancy[gx][gy] += 0.3f;
              if (occupancy[gx][gy] > 1.0f) occupancy[gx][gy] = 1.0f;
            }
          }
        }
      }
    }
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }
}
void smoothHistogram() {
    for (int i = 0; i < NUM_SECTORS; i++) {
    float sum = 0;
    const float WEIGHT_SUM = 9.0f;
    for (int k = -2; k <= 2; k++) {
      int idx = (i + k + NUM_SECTORS) % NUM_SECTORS;
      float w = (k == 0) ? 3.0f : (abs(k) == 1) ? 2.0f : 1.0f;
      sum += w * polar_histogram[idx];
    }
    smoothed_histogram[i] = sum / WEIGHT_SUM;
  }
}
float computeVFH(float targetAngle, float currentTheta) {
    smoothHistogram();
  float relativeTarget = targetAngle - currentTheta;
  while (relativeTarget < 0) relativeTarget += 2 * PI;
  while (relativeTarget >= 2 * PI) relativeTarget -= 2 * PI;
  float bestCost = 1e9;
  float best_chosen_angle = -1.0;
  bool firstFree = smoothed_histogram[0] < VFH_THRESHOLD;
  bool lastFree  = smoothed_histogram[NUM_SECTORS-1] < VFH_THRESHOLD;
  bool merged = false;
  for (int i = 0; i < NUM_SECTORS; ) {
    if (smoothed_histogram[i] >= VFH_THRESHOLD) {
      i++;
      continue;
    }
    int start = i;
    while (i < NUM_SECTORS && smoothed_histogram[i] < VFH_THRESHOLD) {
      i++;
    }
    int end = i - 1;
    if (start == 0 && lastFree && !merged) {
      int j = NUM_SECTORS - 1;
      while (j >= 0 && smoothed_histogram[j] < VFH_THRESHOLD) {
        j--;
      }
      start = j + 1;
      end = i - 1 + NUM_SECTORS;
      merged = true;
    }
    int size = end - start + 1;
    float candidate_angle = 0;
    int s_margin = 3;
    if (size <= s_margin * 2) {
      int mid = (start + end) / 2;
      candidate_angle = (mid % NUM_SECTORS) * 5.0 * PI / 180.0;
    } else {
      int safe_start = start + s_margin;
      int safe_end = end - s_margin;
      float angle_right = (safe_start % NUM_SECTORS) * 5.0 * PI / 180.0;
      float angle_left  = (safe_end % NUM_SECTORS) * 5.0 * PI / 180.0;
      float cost_right = fabs(angle_right - relativeTarget);
      if (cost_right > PI) cost_right = 2 * PI - cost_right;
      float cost_left = fabs(angle_left - relativeTarget);
      if (cost_left > PI) cost_left = 2 * PI - cost_left;
      bool target_inside = false;
      int target_sector = (int)(relativeTarget * 180.0 / PI / 5.0) % NUM_SECTORS;
      if (target_sector >= safe_start && target_sector <= safe_end) {
          target_inside = true;
      }
      else if ((target_sector + NUM_SECTORS) >= safe_start && ((target_sector + NUM_SECTORS) <= safe_end)) {
          target_inside = true;
      }
      if (target_inside) {
        candidate_angle = relativeTarget;
      } else {
        if (cost_right < cost_left) {
           candidate_angle = angle_right;
        } else {
           candidate_angle = angle_left;
        }
      }
    }
    float cost = fabs(candidate_angle - relativeTarget);
    if (cost > PI) cost = 2 * PI - cost;
    if (cost > (120 * PI / 180.0)) continue;
    if (cost < bestCost) {
      bestCost = cost;
      best_chosen_angle = candidate_angle;
    }
  }
  if (best_chosen_angle == -1.0) {
    return -999.0;
  }
  if (best_chosen_angle > PI) best_chosen_angle -= 2 * PI;
  float alpha = 0.6;
  float diff = best_chosen_angle - prevBestAngle;
  while (diff > PI) diff -= 2 * PI;
  while (diff < -PI) diff += 2 * PI;
  float smoothed_angle_rel = prevBestAngle + (1.0 - alpha) * diff;
  while (smoothed_angle_rel > PI) smoothed_angle_rel -= 2 * PI;
  while (smoothed_angle_rel < -PI) smoothed_angle_rel += 2 * PI;
  prevBestAngle = smoothed_angle_rel;
  float safeAngleWorld = currentTheta + smoothed_angle_rel;
  while (safeAngleWorld > PI) safeAngleWorld -= 2 * PI;
  while (safeAngleWorld < -PI) safeAngleWorld += 2 * PI;
  return safeAngleWorld;
}