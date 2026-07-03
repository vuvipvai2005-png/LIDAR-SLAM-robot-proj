#include <Arduino.h>
#include "Lidar_VFH.h"
#include <vector>
#include <Wire.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <SD.h>
#define MPU_ADDR 0x68
#define NUM_SECTORS 72
#define GRID_SIZE 30        
#define CELL_SIZE 33.0f
struct Point { float x; float y; };
float target_x, target_y;
float pwmL=0;
float pwmR=0;
int32_t mode=2;
float mode2=0;
// =========================================================
// THIẾT LẬP FUZZY LOGIC ĐIỀU KHIỂN VẬN TỐC
// =========================================================
#include <math.h>

// Hàm liên thuộc tam giác (Bảo vệ lỗi chia cho 0 giống Python)
float trimf(float x, float a, float b, float c) {
  float left = (b - a != 0.0f) ? (x - a) / (b - a) : ((x == a) ? 1.0f : 0.0f);
  float right = (c - b != 0.0f) ? (c - x) / (c - b) : ((x == b) ? 1.0f : 0.0f);
  return fmax(0.0f, fmin(left, right));
}

// Bảng luật mờ (0: VS, 1: S, 2: M, 3: L)
const int rule_matrix[5][5] = {
  {4, 3, 2, 2, 1}, // y = NL
  {3, 3, 2, 1, 1}, // y = NS
  {3, 2, 2, 1, 1}, // y = Z
  {2, 2, 1, 1, 0}, // y = PS
  {1, 1, 1, 0, 0}  // y = PL
};

// Khai báo tọa độ đỉnh các hàm liên thuộc (a, b, c)
const float theta_params[5][3] = {
  {0, 0, 20},
  {0, 20, 50},
  {20, 50, 80},
  {50, 80, 110},
  {80, 120, 120}
};
const float y_params[5][3]     = {{0,0,0.05}, {0,0.05,0.07}, {0.05,0.07,0.09}, {0.07,0.09,0.12}, {0.09,0.12,0.12}};
const float v_params[5][3] = {
  {0,0,0.08},
  {0,0.08,0.15},
  {0.08,0.15,0.23},
  {0.15,0.23,0.3},
  {0.23,0.3,0.3}
};
// Hàm tính toán Suy diễn MIN-MAX và Giải mờ (CoG)
float computeFuzzyVelocity(float theta_val, float y_val) {
  float theta_fuzz[5], y_fuzz[5];
  
  // Fuzzification (Mờ hóa)
  for(int i = 0; i < 5; i++) {
    theta_fuzz[i] = trimf(theta_val, theta_params[i][0], theta_params[i][1], theta_params[i][2]);
    y_fuzz[i]     = trimf(y_val, y_params[i][0], y_params[i][1], y_params[i][2]);
  }

  // Khởi tạo mảng đồ thị vận tốc (Độ phân giải 50 điểm để tính Trọng tâm)
  const int V_STEPS = 50;
  float aggregated_v[V_STEPS] = {0};
  float v_domain[V_STEPS];
  for(int k = 0; k < V_STEPS; k++) {
    v_domain[k] = k * (0.4f / (V_STEPS - 1)); 
  }

  // Inference (Suy diễn MIN-MAX cắt ngọn tam giác)
  for(int i = 0; i < 5; i++) {       // Lặp qua Y
    for(int j = 0; j < 5; j++) {     // Lặp qua Theta
      float firing_strength = fmin(y_fuzz[i], theta_fuzz[j]);
      if(firing_strength > 0) {
        int out_label = rule_matrix[i][j];
        for(int k = 0; k < V_STEPS; k++) {
          float mf_val = trimf(v_domain[k], v_params[out_label][0], v_params[out_label][1], v_params[out_label][2]);
          float activated_mf = fmin(firing_strength, mf_val);
          aggregated_v[k] = fmax(aggregated_v[k], activated_mf);
        }
      }
    }
  }

  // Defuzzification (Giải mờ Trọng tâm CoG)
  float num = 0, den = 0;
  for(int k = 0; k < V_STEPS; k++) {
    num += v_domain[k] * aggregated_v[k];
    den += aggregated_v[k];
  }

  if(den == 0.0f) return 0.0f; // Chống chia cho 0
  return num / den;
}
#pragma pack(push, 1)

// Gói lệnh ESP32 gửi xuống STM32 (9 Bytes)
typedef struct {
    uint16_t header; // Luôn là 0xAABB để nhận diện
    float vL_cmd;
    float vR_cmd;
    uint8_t checksum;
} ESP_To_STM_Packet;
typedef struct {
    uint16_t header; // 0xEEFF (Để phân biệt với gói kia)
    float param1;
    float param2;
    float param3;
    int32_t mode_int;
    uint8_t checksum;
} ESP_To_STM_Config;
// Gói báo cáo STM32 gửi lên ESP32 (23 Bytes)
typedef struct {
    uint16_t header; // Luôn là 0xCCDD để nhận diện
    float x;
    float y;
    float theta;
    float vL_meas;
    float vR_meas;
  float kpt;         // 4 bytes 
  float kit;         // 4 bytes
  float kdt;
  float kpp;         // 4 bytes 
  float kip;         // 4 bytes
  float kdp;
    uint8_t checksum;
} STM_To_ESP_Packet;

#pragma pack(pop)

// Bê nguyên định nghĩa Struct (ESP_To_STM_Packet và STM_To_ESP_Packet) vào đây
// ... 

ESP_To_STM_Packet tx_cmd;
STM_To_ESP_Packet rx_report;

unsigned long last_uart_time = 0;

uint8_t calculate_checksum(uint8_t* data, uint16_t length) {
    uint8_t sum = 0;
    for (uint16_t i = 0; i < length - 1; i++) sum ^= data[i];
    return sum;
}
void sendConfigToSTM32(float p1, float p2, float p3, int32_t mode) {
    ESP_To_STM_Config cfg_packet;
    cfg_packet.header = 0xEEFF; 
    cfg_packet.param1 = p1;
    cfg_packet.param2 = p2;
    cfg_packet.param3 = p3;
    cfg_packet.mode_int = mode;
    
    cfg_packet.checksum = calculate_checksum((uint8_t*)&cfg_packet, sizeof(ESP_To_STM_Config));
    uint8_t* p = (uint8_t*)&cfg_packet;

    // Bắn thẳng xuống UART 1 lần duy nhất
    Serial1.write((uint8_t*)&cfg_packet, sizeof(ESP_To_STM_Config));
    
   
}


// ========= CONFIG SD CARD =========
const int FLUSH_SAMPLES = 25; 
File dataFile;
int sampleCount = 0;
const int SD_CS_PIN = 5; // Chân CS mặc định
int pwmL_prev=0;
int pwmR_prev=0;
struct __attribute__((packed)) DataPacket {
  uint32_t timestamp;
  float vL;
  float vR;
  float vL_meas;
  float vR_meas;
  float kpt;         // 4 bytes 
  float kit;         // 4 bytes
  float kdt;
  float kpp;         // 4 bytes 
  float kip;         // 4 bytes
  float kdp;
};

// Queue để truyền dữ liệu từ Core 1 (Control) sang Core 0 (SD Logging)
QueueHandle_t sdQueue;
float vL_meas=0;
float vR_meas=0;
float prev_vL=0;
float prev_vR=0;
float offsetx=0;
float offsety=0;
bool isPathReady = false; 
float lookahead = 0.12;
float last_v_cmd = 0;
const float WHEEL_BASE = 0.25;
const float WHEEL_DIAMETER = 0.072;
const float WHEEL_CIRC = PI * WHEEL_DIAMETER;
const int ENCODER_RES = 616*4;
const float TICK_TO_M = WHEEL_CIRC / ENCODER_RES;
float kpt = 0, kit = 0, kdt = 0;
float kpp = 0, kip = 0, kdp = 0;

bool isStopped = true;
int chieu = 0;
int cntt=0;
bool finished = false;

struct Mark { float x; float y; int status; };
std::vector<Point> path;

unsigned long lastTime = 0;
float vL=0;
float vR=0;

const char* ssid = "P203";
const char* password = "203withlove";
const char* udpAddress = "192.168.80.36";
const int udpPort = 12345;
const int udpPortJoy = 12349;
WiFiUDP udp;
WiFiUDP udpJoy; 
struct PacketHeader {
    uint32_t timestamp;
    float x;
    float y;
    float theta;
    uint16_t num_points;
};
#define MAX_POINTS 180
void chuan_hoa(float &x){
  if(x>PI) x-=2*PI;
  else if(x<-PI) x+=2*PI;
}


float integral_ang = 0.0f;
float prevErr_ang = 0.0f;

void taskUDP(void *pvParameters) {
  while (1) {
    float send_ranges[MAX_POINTS];
    uint16_t send_intensity[MAX_POINTS];
    for (int i = 0; i < MAX_POINTS; i++) {
      send_ranges[i] = lidar_dist[i * 2];
      send_intensity[i] = lidar_intensity[i * 2];
    }
    sendScanPacket(x, y, theta, send_ranges, send_intensity, MAX_POINTS);
    vTaskDelay(50 / portTICK_PERIOD_MS);
  }
}
void taskUART(void *pvParameters){
    unsigned long last_tx_time = 0;
    
    while (1) {
        // --- 1. NHẬN DỮ LIỆU TỐC ĐỘ CAO ---
        // Dùng while thay vì if để vét sạch buffer nhanh nhất có thể
        while (Serial1.available() > 0) {
            if (Serial1.peek() != 0xDD) {
                Serial1.read(); 
            } else {
                if (Serial1.available() >= sizeof(STM_To_ESP_Packet)) {
                    Serial1.read(); 
                    
                    if (Serial1.peek() == 0xCC) {
                        uint8_t* ptr = (uint8_t*)&rx_report;
                        ptr[0] = 0xDD;
                        Serial1.readBytes(&ptr[1], sizeof(STM_To_ESP_Packet) - 1); 
                        
                        if (calculate_checksum(ptr, sizeof(STM_To_ESP_Packet)) == rx_report.checksum) {
                            // Chỉ cập nhật biến toàn cục, không làm gì thêm!
                            x = offsetx + rx_report.x;
                            y = offsety + rx_report.y;
                            theta = rx_report.theta;
                            vL_meas = rx_report.vL_meas;
                            vR_meas = rx_report.vR_meas;
                            kpt = rx_report.kpt;
                            kit = rx_report.kit;
                            kdt = rx_report.kdt;
                            kpp = rx_report.kpp;
                            kip = rx_report.kip;
                            kdp = rx_report.kdp;
                        }
                    }
                } else {
                    break; // Chưa đủ 23 bytes, thoát while để chờ
                }
            }
        }

        // --- 2. GỬI LỆNH vL, vR (Mỗi 30ms) ---
        // Task này truyền độc lập, lấy thẳng vL, vR từ biến toàn cục
        unsigned long now = millis();
        if (now - last_tx_time >= 30) {
            last_tx_time = now;
            if(isStopped||finished) {
              if(mode2!=3) sendConfigToSTM32(pwmL,pwmR,mode2,mode);
              else sendConfigToSTM32(pwmL,pwmR,mode2,5);
            }
            else{tx_cmd.header = 0xAABB; 
            tx_cmd.vL_cmd = vL; 
            tx_cmd.vR_cmd = vR;
            tx_cmd.checksum = calculate_checksum((uint8_t*)&tx_cmd, sizeof(ESP_To_STM_Packet));
            
            Serial1.write((uint8_t*)&tx_cmd, sizeof(ESP_To_STM_Packet));
            }
        }
        // Nhường CPU 1ms (Tần số quét UART 1000Hz)
        vTaskDelay(1 / portTICK_PERIOD_MS); 
    }
}


// ========= HÀM KHỞI TẠO SD MẶC ĐỊNH =========
bool initSDLogger(const char* filename = "/data.bin") {
  // Khởi tạo thẻ SD với chân CS = 5, sử dụng bus SPI mặc định ở tốc độ an toàn (4MHz)
  // Nếu muốn chạy tốc độ cao hơn sau này, bạn có thể dùng: SD.begin(SD_CS_PIN, SPI, 20000000)
  if (!SD.begin(SD_CS_PIN)) { 
    Serial.println("LỖI: Không tìm thấy hoặc không thể khởi tạo thẻ SD!");
    return false;
  }
  
  Serial.println("SD OK!");
  
  if (SD.exists(filename)) {
    SD.remove(filename);
  }
  
  dataFile = SD.open(filename, FILE_WRITE);
  if (!dataFile) {
    Serial.println("LỖI: Không thể mở/tạo file ghi dữ liệu!");
    return false;
  }
  
  Serial.println("KHỞI TẠO LOGGING THÀNH CÔNG.");
  return true;
}
void logData(uint32_t t, float vL, float vR, float vL_meas, float vR_meas, float kpt, float kit, float kdt, float kpp, float kip, float kdp) {
  if (!dataFile) return;
  DataPacket p;
  p.timestamp = t;
  p.vL = vL;
  p.vR = vR;
  p.vL_meas = vL_meas;
  p.vR_meas = vR_meas;
  p.kpt = kpt;
  p.kit = kit;
  p.kdt = kdt;
  p.kpp = kpp;
  p.kip = kip;
  p.kdp = kdp;
  dataFile.write((uint8_t*)&p, sizeof(p));
  sampleCount++;
  
  if (sampleCount >= FLUSH_SAMPLES) {
    dataFile.flush();
    sampleCount = 0;
  }
}
// ========= TASK GHI SD (Chạy trên Core 0) =========
// ========= TASK GHI SD (Chạy trên Core 0) =========
void taskSDLog(void *pvParameters) {
  vTaskDelay(2000 / portTICK_PERIOD_MS); 

  if (!initSDLogger()) {
    Serial.println("Task SD Log stopped.");
    vTaskDelete(NULL); 
  }

  DataPacket p;
  while (1) {
    // Chờ nhận data tối đa 1 giây. 
    if (xQueueReceive(sdQueue, &p, 1000 / portTICK_PERIOD_MS)) {
      // Có data -> đem đi ghi
      logData(p.timestamp, p.vL, p.vR, p.vL_meas, p.vR_meas,p.kpt,p.kit,p.kdt,p.kpp,p.kip,p.kdp);
    } else {
      // Quá 1 giây không có data mới (robot dừng / đứng im) -> ÉP LƯU DATA!
      if (sampleCount > 0) {
        dataFile.flush();
        sampleCount = 0;
        Serial.println("[SD] Auto-flushed data!");
      }
    }
  }
}
float calculateCrossTrackError(float x, float y, Point A, Point B) {
    float dx = B.x - A.x;
    float dy = B.y - A.y;

    float path_length = sqrt(dx * dx + dy * dy);

    // Tránh chia 0 (2 waypoint trùng nhau)
    if (path_length < 1e-6f) return 0.0f;

    // Cross-track error có dấu
    float error = (dx * (A.y - y) - (A.x - x) * dy) / path_length;

    return error;
}
float wrapToPi(float a) {
    while (a > PI) a -= 2.0f * PI;
    while (a < -PI) a += 2.0f * PI;
    return a;
}

float wrapTo2Pi(float a) {
    while (a < 0) a += 2.0f * PI;
    while (a >= 2.0f * PI) a -= 2.0f * PI;
    return a;
}

bool isDirectionBlocked(float targetAngleWorld, float currentTheta, float coneDeg = 20.0f) {
    smoothHistogram();

    float rel = wrapTo2Pi(targetAngleWorld - currentTheta);
    int centerSector = (int)((rel * 180.0f / PI) / 5.0f) % NUM_SECTORS;
    int halfWindow = ceil(coneDeg / 5.0f);

    for (int s = -halfWindow; s <= halfWindow; s++) {
        int idx = (centerSector + s + NUM_SECTORS) % NUM_SECTORS;
        if (smoothed_histogram[idx] >= VFH_THRESHOLD) {
            return true;
        }
    }

    return false;
}
bool computeLocalTargetFromVFH(
    float global_tx,
    float global_ty,
    float &local_tx,
    float &local_ty
) {
    float dx = global_tx - x;
    float dy = global_ty - y;
    float dist_to_global = sqrt(dx * dx + dy * dy);

    if (dist_to_global < 0.03f) {
        local_tx = global_tx;
        local_ty = global_ty;
        return true;
    }

    float desiredAngle = atan2(dy, dx);

    bool blocked = isDirectionBlocked(desiredAngle, theta, 20.0f);

    if (!blocked) {
        // Không có vật cản trên hướng global target
        local_tx = global_tx;
        local_ty = global_ty;
        return true;
    }

    // Có vật cản, dùng VFH tìm hướng an toàn
    float safeAngle = computeVFH(desiredAngle, theta);

    if (safeAngle < -900.0f) {
        // Không tìm được khe an toàn
        return false;
    }

    // Không đi thẳng tới global target nữa,
    // mà tạo một target ngắn theo hướng VFH
    float localLookahead = 0.25f; // mét, nên chỉnh 0.2 - 0.4 m

    if (dist_to_global < localLookahead) {
        localLookahead = dist_to_global;
    }

    local_tx = x + localLookahead * cos(safeAngle);
    local_ty = y + localLookahead * sin(safeAngle);

    return true;
}
void controlLoop() {
    float ctrl_tx = target_x;
    float ctrl_ty = target_y;

    // Local planner VFH nằm ở đây
    bool local_ok = computeLocalTargetFromVFH(target_x, target_y, ctrl_tx, ctrl_ty);

    if (!local_ok) {
        // Không có khe an toàn -> dừng lại mềm mại
        vL = vL * 0.5f; 
        vR = vR * 0.5f;
        if(abs(vL) < 0.01f && abs(vR) < 0.01f) { vL = 0; vR = 0; }
        return;
    }

    // --- THÊM LỌC LOW-PASS CHO LOCAL TARGET ĐỂ CHỐNG VFH PING-PONG ---
    static float smoothed_ctrl_tx = x;
    static float smoothed_ctrl_ty = y;
    
    // Khởi tạo nếu xe vừa bắt đầu chạy
    if (isStopped || finished) {
        smoothed_ctrl_tx = ctrl_tx;
        smoothed_ctrl_ty = ctrl_ty;
    } else {
        // Tin 20% target mới (VFH), giữ 80% target cũ để xe bẻ lái mượt mà
        smoothed_ctrl_tx = smoothed_ctrl_tx * 0.8f + ctrl_tx * 0.2f;
        smoothed_ctrl_ty = smoothed_ctrl_ty * 0.8f + ctrl_ty * 0.2f;
    }

    float slow_down_dist = 0.2f;
    float min_v = 0.05f;
    
    // Tính toán theo target đã được làm mượt
    float dx = smoothed_ctrl_tx - x;
    float dy = smoothed_ctrl_ty - y;
    float dist = sqrt(dx*dx + dy*dy);
    
    // Kiểm tra hoàn thành mục tiêu gốc
    float global_dx = target_x - x;
    float global_dy = target_y - y;
    float global_dist = sqrt(global_dx*global_dx + global_dy*global_dy);

    if (global_dist < 0.03f) {
        vL = 0;
        vR = 0;
        finished = true;
        return;
    }

    float scale = constrain(global_dist / slow_down_dist, 0.0f, 1.0f);
    scale = scale * scale; // mượt hơn
    
    float targetAngle = atan2(dy, dx);
    float angleError = targetAngle - theta;
    chuan_hoa(angleError);
    float y_err = 0.01f;
    float angle_err_deg = angleError * 180.0f / PI;
    
    float v = computeFuzzyVelocity(abs(angle_err_deg), abs(y_err));
    v = v * scale;
    v = max(v, min_v);

    // --- SỬA LỖI TOÁN HỌC GÂY RUNG LẮC Ở ĐÂY ---
    // Giới hạn dist_sq tối thiểu để không bị bùng nổ w khi chia cho số quá gần 0
    float dist_sq = max(dist * dist, 0.01f); 
    float w = v * 2 * (-sin(theta)*dx + cos(theta)*dy) / dist_sq;
    
    // BẮT BUỘC PHẢI GIỚI HẠN w (Ví dụ: tối đa 1.5 rad/s - tùy xe của bạn)
    w = constrain(w, -1.5f, 1.5f); 

    // 2. TÍNH TOÁN vL, vR THÔ
    float raw_vL = v - w * WHEEL_BASE / 2.0f;
    float raw_vR = v + w * WHEEL_BASE / 2.0f;

    // 3. LỌC LOW-PASS Ở CHỐT CHẶN CUỐI CÙNG
    static float prev_cmd_vL = 0.0f;
    static float prev_cmd_vR = 0.0f;

    // Tăng nhẹ quán tính (từ 0.5 lên 0.7) nếu xe thực tế vẫn còn hơi gắt
    vL = raw_vL * 0.3f + prev_cmd_vL * 0.7f;
    vR = raw_vR * 0.3f + prev_cmd_vR * 0.7f;
    
    prev_cmd_vL = vL;
    prev_cmd_vR = vR;

    // Log dữ liệu SD
    DataPacket logPacket;
    logPacket.timestamp = millis();
    logPacket.vL = vL;
    logPacket.vR = vR;
    logPacket.vL_meas = vL_meas;
    logPacket.vR_meas = vR_meas;
    logPacket.kpt = kpt;
    logPacket.kit = kit;
    logPacket.kdt = kdt;
    logPacket.kpp = kpp;
    logPacket.kip = kip;
    logPacket.kdp = kdp;
    xQueueSend(sdQueue, &logPacket, 0); 
}
// --- TASK TÍNH TOÁN ĐIỀU KHIỂN (Chạy chuẩn xác mỗi 30ms) ---
void taskControl(void *pvParameters) {
    // Khởi tạo bộ đếm nhịp của FreeRTOS
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = 30 / portTICK_PERIOD_MS; // Chu kỳ 30ms

    while (1) {
        if (!isStopped && !finished) {
        
            controlLoop(); 
           // cntt++;
            
        } else {
            vL = 0;
            vR = 0;
        }
        
        // Ngủ chính xác cho đến nhịp 30ms tiếp theo
        vTaskDelayUntil(&xLastWakeTime, xFrequency); 
    }
}

void initWiFi() {
  WiFi.begin(ssid, password);
  Serial.print("Connecting to WiFi");
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConnected!");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
  udp.begin(12346);
  udpJoy.begin(12349);
}
void sendScanPacket(float cur_x, float cur_y, float cur_theta, float* ranges, uint16_t* intensities, uint16_t num_points) {
  if (WiFi.status() != WL_CONNECTED) return;
  uint8_t sync[2] = {0xAA, 0x55};
  PacketHeader header;
  header.timestamp = millis();
  header.x = cur_x;
  header.y = cur_y;
  header.theta = cur_theta;
  header.num_points = num_points;
  if (udp.beginPacket(udpAddress, udpPort) != 1) return;
  udp.write(sync, 2);
  udp.write((uint8_t*)&header, sizeof(PacketHeader));
  udp.write((uint8_t*)ranges, num_points * sizeof(float));
  udp.write((uint8_t*)intensities, num_points * sizeof(uint16_t));
  uint8_t checksum = 0;
  uint8_t* ptr = (uint8_t*)&header;
  for (int i = 0; i < sizeof(PacketHeader); i++) checksum ^= ptr[i];
  ptr = (uint8_t*)ranges;
  for (int i = 0; i < num_points * sizeof(float); i++) checksum ^= ptr[i];
  ptr = (uint8_t*)intensities;
  for (int i = 0; i < num_points * sizeof(uint16_t); i++) checksum ^= ptr[i];
  udp.write(checksum);
  udp.endPacket();
}
void setup() {
  Serial.begin(115200);
  Serial1.begin(921600, SERIAL_8N1, 16, 17); // RX=16, TX=17
  initWiFi();
  beginLidar();
  for (int i = 0; i < 360; i++) {
    lidar_dist[i] = 10.0;
  }
  sdQueue = xQueueCreate(100, sizeof(DataPacket));
  // Chạy task ghi thẻ SD trên Core 0 (cùng core với Lidar, né Core 1 của PID)
// Core 0: Gánh phần cứng (SD, WiFi UDP, Lidar)
  xTaskCreatePinnedToCore(taskSDLog, "TaskSD", 8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskLidar, "TaskLidar", 8192, NULL, 1, NULL, 0);
  xTaskCreatePinnedToCore(taskUDP, "UDP", 8192, NULL, 1, NULL, 0);
  
  // Core 1: Gánh Não bộ (Toán học và Giao tiếp thời gian thực)
  // Ưu tiên cao hơn (Priority 2) cho việc đọc UART để không trượt byte
  xTaskCreatePinnedToCore(taskUART, "TaskUART", 8192, NULL, 2, NULL, 1);
  // Ưu tiên 1 cho Control Loop
  xTaskCreatePinnedToCore(taskControl, "TaskControl", 8192, NULL, 1, NULL, 1);
  lastTime = millis();
}
// Thêm biến này ở khu vực khai báo biến toàn cục (cùng chỗ với isStopped, mode...)


void checkIncomingCommand() {
  int packetSize;
  
  // VÒNG LẶP VÉT BỘ ĐỆM: Kéo sạch toàn bộ gói tin đang chờ ra xử lý lập tức
  while ((packetSize = udp.parsePacket()) > 0) {
  if (packetSize == 11) {
    uint8_t buf[11];
    udp.read(buf, 11);

    if (buf[0] == 0xBB && buf[1] == 0x66) {
        
        memcpy(&target_x, &buf[2], 4);
        memcpy(&target_y, &buf[6], 4);

        uint8_t chk = 0;
        for(int i = 2; i < 10; i++) chk ^= buf[i];

        if (chk == buf[10]&& mode2 != 3) {
            finished = false;
            isStopped = false;
        }
    }
}
    else {
        udp.flush();
    }
  }
}

void receiveJoystick() {
  int packetSize = udpJoy.parsePacket();
  if (packetSize == 7) {   // <-- sửa 5 -> 7
    uint8_t buf[7];
    udpJoy.read(buf, 7);

    if (buf[0] == 0xAA) {
      int16_t x, y;
      int8_t modee, chosen;

      memcpy(&x, &buf[1], 2);
      memcpy(&y, &buf[3], 2);
      modee = (int8_t)buf[5];
      chosen = (int8_t)buf[6];

      // scale về -1 → 1 (giống Python)
      pwmL = x;
      pwmR = y;
      mode = (int32_t)chosen;
      mode2 = (float)modee;
      // debug
    }
  }
}
void loop() {
    checkIncomingCommand(); // Lắng nghe UDP từ App
    receiveJoystick();
    if(mode2==3) {
      finished=true;
      isStopped=true;
    }
    delay(10);
}
