/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdbool.h>
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
// Địa chỉ I2C của MPU6050 trên STM32 phải được dịch trái 1 bit (0x68 << 1 = 0xD0)
#define MPU_ADDR 0xD0
#define DEG2RAD 0.01745329251f
#define PI 3.14159265358979323846f
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

I2C_HandleTypeDef hi2c1;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;

UART_HandleTypeDef huart1;
DMA_HandleTypeDef handle_GPDMA1_Channel1;
DMA_HandleTypeDef handle_GPDMA1_Channel0;
float hihi=0;
/* USER CODE BEGIN PV */
#pragma pack(push, 1)

typedef struct {
    uint16_t header;  // 2 bytes (0xAABB)
    float vL_cmd;     // 4 bytes
    float vR_cmd;     // 4 bytes
    uint8_t checksum; // 1 byte
} ESP_To_STM_Packet;  // TỔNG: 11 Bytes

typedef struct {
    uint16_t header;  // 2 bytes (0xCCDD)
    float x;          // 4 bytes
    float y;          // 4 bytes
    float theta;      // 4 bytes
    float vL_meas;    // 4 bytes
    float vR_meas;
    float kpt;         // 4 bytes
    float kit;         // 4 bytes
    float kdt;
    float kpp;         // 4 bytes
    float kip;         // 4 bytes
    float kdp;
    uint8_t checksum; // 1 byte
} STM_To_ESP_Packet;  // TỔNG: 35 Bytes
typedef struct {
    uint16_t header; // 0xEEFF
    float param1;
    float param2;
    float param3;
    int32_t mode_int;
    uint8_t checksum;
} ESP_To_STM_Config;
typedef enum {
    PACKET_NONE,
    PACKET_REALTIME, // 0xAABB
    PACKET_CONFIG    // 0xEEFF
} PacketType_t;

PacketType_t current_packet_type = PACKET_NONE;
uint8_t expected_length = 0; // Biến lưu độ dài gói tin đang hứng
#define RBF_NODES 5 // 5 nơ-ron là mức cân bằng hoàn hảo giữa độ chính xác và tốc độ
static inline float constrain_f(float x, float min, float max)
{
    if (x < min) return min;
    if (x > max) return max;
    return x;
}
float max_f(float a, float b) {
    return (a > b) ? a : b;
}
// Cấu trúc lưu trữ toàn bộ trạng thái của một bộ điều khiển RBF-PID
typedef struct {
    // 1. Mạng RBF
    float w[RBF_NODES];       // Trọng số (Weights)
    float w_prev[RBF_NODES];  // Trọng số bước trước (Dùng cho Momentum)
    float c[3][RBF_NODES];    // Tâm Gaussian (Centers) - 3 ngõ vào: u(k-1), y(k-1), y(k-2)
    float b[RBF_NODES];       // Độ rộng Gaussian (Widths)

    // 2. Hệ số PID hiện tại
    float Kp, Ki, Kd;

    // 3. Tốc độ học (Learning Rates) và Động lượng
    float eta_w;              // Tốc độ học của RBF
    float eta_kp, eta_ki, eta_kd; // Tốc độ học của PID
    float alpha;              // Động lượng (Momentum)

    // 4. Biến trạng thái trễ (Delays)
    float u_prev;             // Tín hiệu điều khiển u(k-1)
    float y_prev, y_prev2;    // Vận tốc y(k-1), y(k-2)
    float e_prev, e_prev2;    // Sai số e(k-1), e(k-2)

    // 5. Giới hạn an toàn (Clamping)
    float Kp_max, Ki_max, Kd_max;
    float u_max;              // Max PWM (ví dụ: 1000)
    float max_vel;            // Vận tốc tối đa để chuẩn hóa ngõ vào (ví dụ: 2.0 m/s)
} RBF_PID_Controller;

// Khai báo 2 bộ điều khiển cho 2 bánh
RBF_PID_Controller pid_left;
RBF_PID_Controller pid_right;
void RBF_PID_Init(RBF_PID_Controller *ctrl, float init_Kp, float init_Ki, float init_Kd, float max_pwm, float max_v) {
    // 1. Khởi tạo PID ban đầu
    ctrl->Kp = init_Kp;
    ctrl->Ki = init_Ki;
    ctrl->Kd = init_Kd;

    // 2. GIỮ LẠI 2 DÒNG NÀY (Tuyệt đối không được xóa)
    ctrl->u_max   = max_pwm;
    ctrl->max_vel = max_v;

    // 3. Giới hạn an toàn
    ctrl->Kp_max = 8000.0f;
    ctrl->Ki_max = 4000.0f;  // Ki đã quy đổi gia số, max chỉ cần 100
    ctrl->Kd_max = 500.0f;

    // 4. Tốc độ học và Động lượng (Chuẩn cho PID Gia Số)
    // Tốc độ học (Đã bật chế độ TURBO để Kp, Ki, Kd múa trên đồ thị)
        ctrl->eta_w  = 0.1f;
        ctrl->eta_kp = 180000.0f; // Tăng lên 50 nghìn! (Vì xc_1 quá nhỏ)
        ctrl->eta_ki = 4000.0f;   // Tăng lên 500
        ctrl->eta_kd = 12000.0f;  // Tăng lên 5 nghìn!
        ctrl->alpha  = 0.15f;

    // 5. Khởi tạo biến trạng thái
    ctrl->u_prev = 0;
    ctrl->y_prev = 0; ctrl->y_prev2 = 0;
    ctrl->e_prev = 0; ctrl->e_prev2 = 0;

    // 6. Khởi tạo mạng RBF
    float center_spread[RBF_NODES] = {-1.0f, -0.5f, 0.0f, 0.5f, 1.0f};

    for (int j = 0; j < RBF_NODES; j++) {
        ctrl->w[j] = 0.1f;
        ctrl->w_prev[j] = 0.1f;
        ctrl->b[j] = 0.5f;

        ctrl->c[0][j] = center_spread[j];
        ctrl->c[1][j] = center_spread[j];
        ctrl->c[2][j] = center_spread[j];
    }
}
float RBF_PID_Compute(RBF_PID_Controller *ctrl, float setpoint, float measurement) {
    // 1. Chuẩn hóa ngõ vào [-1, 1] để mạng RBF học ổn định
    float x[3];
    x[0] = ctrl->u_prev / ctrl->u_max;
    x[1] = ctrl->y_prev / ctrl->max_vel;
    x[2] = ctrl->y_prev2 / ctrl->max_vel;

    // 2. Mạng RBF Tính toán Forward Pass
    float h[RBF_NODES];
    float ym = 0.0f; // Vận tốc dự đoán
    float dyu = 0.0f; // Jacobian (độ nhạy dY/dU)

    for (int j = 0; j < RBF_NODES; j++) {
        float dist_sq = 0.0f;
        for (int i = 0; i < 3; i++) {
            float diff = x[i] - ctrl->c[i][j];
            dist_sq += diff * diff;
        }
        // Hàm Gauss (FPU optimized)
        h[j] = expf(-dist_sq / (2.0f * ctrl->b[j] * ctrl->b[j]));
        ym += ctrl->w[j] * h[j];
    }

    // ... (Giữ nguyên phần 1 và 2) ...

        // 3. Tính toán Sai số nhận dạng & Jacobian
        float em = (measurement / ctrl->max_vel) - ym;

        for (int j = 0; j < RBF_NODES; j++) {
            dyu += ctrl->w[j] * h[j] * (ctrl->c[0][j] - x[0]) / (ctrl->b[j] * ctrl->b[j]);
        }

        // Ràng buộc đạo hàm & ÉP LUÔN DƯƠNG (Cực kỳ quan trọng cho động cơ)
        // Tăng PWM chắc chắn tăng tốc -> dyu phải > 0
        float abs_dyu = fabsf(dyu);
        if (abs_dyu < 0.5f) abs_dyu = 0.5f; // Không cho phép bằng 0 để tránh đứng im
        if (abs_dyu > 10.0f) abs_dyu = 10.0f;

        // 4. Backpropagation: Cập nhật Trọng số Mạng (Có Momentum)
        for (int j = 0; j < RBF_NODES; j++) {
            float d_w = ctrl->eta_w * em * h[j];
            float w_new = ctrl->w[j] + d_w + ctrl->alpha * (ctrl->w[j] - ctrl->w_prev[j]);
            ctrl->w_prev[j] = ctrl->w[j];
            ctrl->w[j] = w_new;
        }

        // 5. Tính toán sai số điều khiển và Vector Đạo hàm (xc)
        float error = setpoint - measurement;
        float xc_1 = error - ctrl->e_prev;                       // cho Kp
        float xc_2 = error;                                      // cho Ki (PID Gia số)
        float xc_3 = error - 2.0f * ctrl->e_prev + ctrl->e_prev2;// cho Kd

        // 6. Adaptive: Cập nhật Kp, Ki, Kd (KÈM VÙNG CHẾT DEADBAND)
        // Chỉ cập nhật thông số khi sai số > 0.01 m/s. Đạt mục tiêu rồi thì giữ nguyên!
        if (fabsf(error) > 0.00005f) {
            // Chú ý: Dùng abs_dyu thay vì dyu
            float delta_kp = ctrl->eta_kp * error * abs_dyu * xc_1;
            float delta_ki = ctrl->eta_ki * error * abs_dyu * xc_2;
            float delta_kd = ctrl->eta_kd * error * abs_dyu * xc_3;

            // Khóa Delta an toàn: Không cho phép Kp, Ki nhảy quá 20 đơn vị mỗi 15ms
            // Nới lỏng Delta: Cho phép Kp tăng/giảm tối đa 150 đơn vị mỗi 15ms
                    delta_kp = constrain_f(delta_kp, -3000.0f, 3000.0f);
                    delta_ki = constrain_f(delta_ki, -500.0f, 500.0f);
                    delta_kd = constrain_f(delta_kd, -500.0f, 500.0f);

            ctrl->Kp += delta_kp;
            ctrl->Ki += delta_ki;
            ctrl->Kd += delta_kd;
        }

        // Clamping: Đảm bảo Kp, Ki, Kd luôn dương và nằm trong giới hạn
        ctrl->Kp = constrain_f(ctrl->Kp, 0.0f, ctrl->Kp_max);
        ctrl->Ki = constrain_f(ctrl->Ki, 0.0f, ctrl->Ki_max);
        ctrl->Kd = constrain_f(ctrl->Kd, 0.0f, ctrl->Kd_max);

        // ... (Giữ nguyên phần 7 tính ngõ ra du và phần 8) ...
    // 7. Tính ngõ ra PID Gia số (Tính ra delta U)
    float du = ctrl->Kp * xc_1 + ctrl->Ki * xc_2 + ctrl->Kd * xc_3;
    float u = ctrl->u_prev + du; // u(k) = u(k-1) + delta_U

    // Giới hạn PWM cứng
    if (u > ctrl->u_max) u = ctrl->u_max;
    if (u < -ctrl->u_max) u = -ctrl->u_max;

    // 8. Cập nhật biến trễ cho chu kỳ sau
    ctrl->e_prev2 = ctrl->e_prev;
    ctrl->e_prev = error;
    ctrl->y_prev2 = ctrl->y_prev;
    ctrl->y_prev = measurement;
    ctrl->u_prev = u;

    return u;
}
#pragma pack(pop)

// --- BIẾN CHO MÁY TRẠNG THÁI (STATE MACHINE) ---
typedef enum {
    WAIT_HEADER_1,
    WAIT_HEADER_2,
    READ_PAYLOAD
} RX_State_t;

RX_State_t rx_state = WAIT_HEADER_1;
uint8_t rx_buffer[25];   // Buffer hứng từng byte
uint8_t rx_index = 0;
uint8_t rx_byte_tmp;     // Biến hứng 1 byte từ UART

// Các biến điều khiển cũ...
float current_vL_cmd = 0.0f;
float current_vR_cmd = 0.0f;
STM_To_ESP_Packet tx_report;

// Hàm tính Checksum
uint8_t calculate_checksum(uint8_t* data, uint16_t length) {
    uint8_t sum = 0;
    for (uint16_t i = 0; i < length - 1; i++) sum ^= data[i];
    return sum;
}

const float WHEEL_BASE = 0.25;
const float WHEEL_DIAMETER = 0.072;
const float WHEEL_CIRC = PI * WHEEL_DIAMETER;
const int ENCODER_RES = 616*4;
const float TICK_TO_M = WHEEL_CIRC / ENCODER_RES;
const float MAX_INTEGRAL = 1000.0f;
uint32_t last_rx_time = 0;
// Biến lưu Bias (Offset)
float gyroBiasX = 0.0f, gyroBiasY = 0.0f, gyroBiasZ = 0.0f;

// Biến toàn cục cho thuật toán Madgwick
float beta_madgwick = 0.08f;
float q0 = 1.0f, q1 = 0.0f, q2 = 0.0f, q3 = 0.0f;

// Biến lưu góc Euler
float imu_yaw = 0.0f, imu_pitch = 0.0f, imu_roll = 0.0f;

// Biến thời gian cho IMU
uint32_t last_imu_time = 0;
uint32_t last_enc_time = 0;

// Biến lưu giá trị đếm tuyệt đối của Timer
uint32_t current_left_ticks = 0;
uint32_t current_right_ticks = 0;

uint32_t prev_left_ticks = 0;
uint32_t prev_right_ticks = 0;

// Biến lưu độ chênh lệch xung (Delta) để gửi cho ESP32 hoặc chạy PID
int32_t delta_left = 0;  // Có thể mang dấu âm nếu đi lùi
int32_t delta_right = 0;

ESP_To_STM_Packet rx_cmd;
STM_To_ESP_Packet tx_report;

uint32_t last_pid_time = 0;
uint32_t last_uart_time = 0;
uint32_t last_pwm_time = 0;
uint32_t last_odo_time = 0;
float vR_meas=0;
float vL_meas=0;
float prev_vL_meas = 0;
float prev_vR_meas = 0;
float prevErrL = 0, prevErrR = 0;
float intL = 0, intR = 0;
float x,y,theta =0;
const float MAX_E = 0.3f;
const float MAX_DE = 9.0f;
float kpt = 3700.0f, kit = 3000.0f, kdt = 20.0f;
float kpp = 3700.0f, kip =  3000.0f, kdp =20.0f;
float kpt_fuz = 3591.54f, kit_fuz = 1999.97f;
float kpp_fuz = 3543.55f, kip_fuz =  1621.23f;
float pwml1 = 0, pwmr1 = 0;
float pwmL_prev, pwmR_prev = 0;
float pwm_MNL=0.0f;
float pwm_MNR=0.0f;
uint32_t mode=3;
// 2. Cấu hình giới hạn đầu ra (Singleton)
// Ngõ ra: 0=Z (Zero), 1=S (Small), 2=M (Medium), 3=B (Big)
const float Kp_out[4] = {0.0f, 1000.0f, 3000.0f, 4000.0f};
const float Ki_out[4] = {0.0f, 1700.0f, 3400.0f, 5000.0f};

// 3. Bảng luật mờ (Đã ánh xạ từ phân tích: 0=Z, 1=S, 2=M, 3=B)
// Hàng: e (NB, NS, ZE, PS, PB) | Cột: de (NB, NS, ZE, PS, PB)
const int rule_Kp[5][5] = {
  {3, 3, 3, 2, 1}, // e = NB
  {3, 3, 2, 1, 1}, // e = NS
  {2, 2, 1, 2, 2}, // e = ZE
  {1, 1, 2, 3, 3}, // e = PS
  {1, 2, 3, 3, 3}  // e = PB
};

const int rule_Ki[5][5] = {
  {0, 0, 0, 0, 0}, // e = NB
  {0, 1, 1, 2, 0}, // e = NS
  {1, 2, 3, 2, 1}, // e = ZE
  {0, 2, 1, 1, 0}, // e = PS
  {0, 0, 0, 0, 0}  // e = PB
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_GPDMA1_Init(void);
static void MX_I2C1_Init(void);
static void MX_ICACHE_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM5_Init(void);
static void MX_TIM6_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */
/* USER CODE BEGIN PFP */

void MPU6050_Init(void);
bool readIMU(float *ax, float *ay, float *az, float *gx, float *gy, float *gz);
void calibrateGyroBias(int samples);
void Madgwick_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt);
void Compute_Euler_Angles(void);
void Read_Encoder_Delta(void);
void setMotorLeft(int pwm);
void setMotorRight(int pwm);
void computeFuzzyPI(float e, float de, float *Kp_fuzzy, float *Ki_fuzzy);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint8_t rx_byte; // Biến lưu 1 byte dữ liệu nhận được
uint8_t tx_msg[] = "STM32H5 san sang giao tiep!\r\n";

void chuan_hoa(float *x){
  if(*x>PI) *x-=2*PI;
  else if(*x<-PI) *x+=2*PI;
}
float PIDLeft(float error, float *prevErr, float *integral, float antiwindup, float dt) {
  // 1. Tính toán de (đạo hàm sai số)
  float derivative = (error - *prevErr) / dt;

  // 3. Tính toán Tích phân (Integral) với Ki thay đổi linh hoạt
  *integral += error * dt;

  // Áp dụng Clamping cho thành phần I (Chống Windup cứng)
  float I_term = kit * (*integral);
  if (I_term > MAX_INTEGRAL) {
      I_term = MAX_INTEGRAL;
      *integral = MAX_INTEGRAL / (kit + 1e-6); // Cập nhật lại I cốt lõi
  } else if (I_term < -MAX_INTEGRAL) {
      I_term = -MAX_INTEGRAL;
      *integral = -MAX_INTEGRAL / (kit + 1e-6);
  }

  // 4. Tính toán Ngõ ra (Kdt cố định do bạn set trên App/UDP)
  float out = (kpt * error) + I_term + (kdt * derivative);
out=4.0f*out;
  *prevErr = error;
  return out;
}

float PIDRight(float error, float *prevErr, float *integral, float antiwindup, float dt) {
  // 1. Tính toán de (đạo hàm sai số)
  float derivative = (error - *prevErr) / dt;

  // 3. Tính toán Tích phân (Integral)
  *integral += error * dt;

  // Clamping
  float I_term = kip * (*integral);
  if (I_term > MAX_INTEGRAL) {
      I_term = MAX_INTEGRAL;
      *integral = MAX_INTEGRAL / (kip + 1e-6);
  } else if (I_term < -MAX_INTEGRAL) {
      I_term = -MAX_INTEGRAL;
      *integral = -MAX_INTEGRAL / (kip + 1e-6);
  }

  // 4. Tính toán Ngõ ra (Kdp cố định)
  float out = (kpp * error) + I_term + (kdp * derivative);
  out=4.0f*out;
  *prevErr = error;
  return out;
}
float PID_FuzLeft(float error, float *prevErr, float *integral, float antiwindup, float dt) {
  // 1. Tính toán de (đạo hàm sai số)
  float derivative = (error - *prevErr) / dt;

  // 2. Lấy Kp, Ki từ Fuzzy Logic
 // float dynamic_Kp, dynamic_Ki;
  computeFuzzyPI(error, derivative, &kpt_fuz, &kit_fuz);

  // 3. Tính toán Tích phân (Integral) với Ki thay đổi linh hoạt
  *integral += error * dt;

  // Áp dụng Clamping cho thành phần I (Chống Windup cứng)
  float I_term = kit_fuz * (*integral);
  if (I_term > MAX_INTEGRAL) {
      I_term = MAX_INTEGRAL;
      *integral = MAX_INTEGRAL / (kit_fuz + 1e-6); // Cập nhật lại I cốt lõi
  } else if (I_term < -MAX_INTEGRAL) {
      I_term = -MAX_INTEGRAL;
      *integral = -MAX_INTEGRAL / (kit_fuz + 1e-6);
  }

  // 4. Tính toán Ngõ ra (Kdt cố định do bạn set trên App/UDP)
  float out = (kpt_fuz * error) + I_term + (15 * derivative);
out=4.0f*out;
  *prevErr = error;
  return out;
}

float PID_FuzRight(float error, float *prevErr, float *integral, float antiwindup, float dt) {
  // 1. Tính toán de (đạo hàm sai số)
  float derivative = (error - *prevErr) / dt;

  // 2. Lấy Kp, Ki từ Fuzzy Logic
  //float dynamic_Kp, dynamic_Ki;
  computeFuzzyPI(error, derivative, &kpp_fuz, &kip_fuz);

  // 3. Tính toán Tích phân (Integral)
  *integral += error * dt;

  // Clamping
  float I_term = kip_fuz * (*integral);
  if (I_term > MAX_INTEGRAL) {
      I_term = MAX_INTEGRAL;
      *integral = MAX_INTEGRAL / (kip_fuz + 1e-6);
  } else if (I_term < -MAX_INTEGRAL) {
      I_term = -MAX_INTEGRAL;
      *integral = -MAX_INTEGRAL / (kip_fuz + 1e-6);
  }

  // 4. Tính toán Ngõ ra (Kdp cố định)
  float out = (kpp_fuz * error) + I_term + (15 * derivative);
  out=4.0f*out;
  *prevErr = error;
  return out;
}
float GAPIDLeft(float error, float *prevErr, float *integral, float antiwindup, float dt) {
  // 1. Tính toán de (đạo hàm sai số)
  float derivative = (error - *prevErr) / dt;

  // 2. Lấy Kp, Ki từ Fuzzy Logic
 // float dynamic_Kp, dynamic_Ki;
  //computeFuzzyPI(error, derivative, &kpt, &kit);

  // 3. Tính toán Tích phân (Integral) với Ki thay đổi linh hoạt
  *integral += error * dt;

  // Áp dụng Clamping cho thành phần I (Chống Windup cứng)
  float I_term = 1999.97 * (*integral);
  if (I_term > MAX_INTEGRAL) {
      I_term = MAX_INTEGRAL;
      *integral = MAX_INTEGRAL / (1999.97 + 1e-6); // Cập nhật lại I cốt lõi
  } else if (I_term < -MAX_INTEGRAL) {
      I_term = -MAX_INTEGRAL;
      *integral = -MAX_INTEGRAL / (1999.97 + 1e-6);
  }

  // 4. Tính toán Ngõ ra (Kdt cố định do bạn set trên App/UDP)
  float out = (3591.54 * error) + I_term + (10 * derivative);
out=4.0f*out;
  *prevErr = error;
  return out;
}

float GAPIDRight(float error, float *prevErr, float *integral, float antiwindup, float dt) {
  // 1. Tính toán de (đạo hàm sai số)
  float derivative = (error - *prevErr) / dt;

  // 2. Lấy Kp, Ki từ Fuzzy Logic
  //float dynamic_Kp, dynamic_Ki;
  //computeFuzzyPI(error, derivative, &kpp, &kip);

  // 3. Tính toán Tích phân (Integral)
  *integral += error * dt;

  // Clamping
  float I_term = 1998.3 * (*integral);
  if (I_term > MAX_INTEGRAL) {
      I_term = MAX_INTEGRAL;
      *integral = MAX_INTEGRAL / (1998.3 + 1e-6);
  } else if (I_term < -MAX_INTEGRAL) {
      I_term = -MAX_INTEGRAL;
      *integral = -MAX_INTEGRAL / (1998.3 + 1e-6);
  }

  // 4. Tính toán Ngõ ra (Kdp cố định)
  float out = (2317.09 * error) + I_term + (10.51 * derivative);
  out=4.0f*out;
  *prevErr = error;
  return out;
}
float Kp_ang = 2.0f;
float Ki_ang = 0.02f;
float Kd_ang = 0.1f;
float integral_ang = 0.0f;
float prevErr_ang = 0.0f;
float anglePID(float ErrAngle, float dt) {
  // Chuẩn hóa góc về [-PI, PI]
  float err = abs(ErrAngle);
  while (err > PI) err -= 2 * PI;
  // PID cơ bản
  integral_ang += err * dt;
  float derivative = (err - prevErr_ang) / dt;
  prevErr_ang = err;

  float output = Kp_ang * err + Ki_ang * integral_ang + Kd_ang * derivative;

  // Giới hạn tốc độ quay mong muốn
  output = constrain(output, 0.4f, 0.8f); // rad/s

  return abs(output);
}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
	/* USER CODE BEGIN PV */


	/* USER CODE END PV */
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_GPDMA1_Init();
  MX_I2C1_Init();
  MX_ICACHE_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM6_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */


    // Kích hoạt chức năng đếm Encoder trên tất cả các kênh (TI1 và TI2)
    HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
    HAL_TIM_Encoder_Start(&htim5, TIM_CHANNEL_ALL);
    /* USER CODE BEGIN 2 */

      // Khởi động phát xung PWM trên TIM3 kênh 1 và kênh 2
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
      HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
      // Cấu trúc: (&struct, Kp_neo, Ki_neo, Kd_neo, Max_PWM, Max_Vel)
            RBF_PID_Init(&pid_left, 4000.0f, 180.0f, 133.0f, 1000.0f, 1.5f);
            RBF_PID_Init(&pid_right, 4000.0f, 180.0f, 133.0f, 1000.0f, 1.5f);
      MPU6050_Init();

      /* USER CODE BEGIN 2 */
            // ... (khởi động PWM, TIM, MPU) ...
            calibrateGyroBias(400);
            __HAL_UART_CLEAR_OREFLAG(&huart1);
                  __HAL_UART_CLEAR_NEFLAG(&huart1);
                  __HAL_UART_CLEAR_FEFLAG(&huart1);
                  volatile uint32_t dummy = huart1.Instance->RDR; // Đọc bỏ byte rác (dùng ->DR nếu là dòng F1/F4)
                  (void)dummy;
           // HAL_UART_Receive_DMA(&huart1, (uint8_t*)&rx_cmd, sizeof(ESP_To_STM_Packet));
                  HAL_UART_Receive_DMA(&huart1, &rx_byte_tmp, 1);
            uint32_t now = HAL_GetTick();
            last_imu_time = now;
            last_enc_time = now;
            last_odo_time = now;
            last_pid_time = now;
            last_uart_time = now;

        /* USER CODE END 2 */

        /* Infinite loop */
        /* USER CODE BEGIN WHILE */
        while (1)
        {
            now = HAL_GetTick();
            volatile uint8_t debug_cfg_size = sizeof(ESP_To_STM_Config);
            // ==========================================
            // TASK 1: ĐỌC IMU (10ms - 100Hz)
            // ==========================================
            if ((now - last_imu_time) >= 10) {
                float dt_imu = (now - last_imu_time) / 1000.0f;
                last_imu_time = now;

                float ax, ay, az, gx, gy, gz;
                if (readIMU(&ax, &ay, &az, &gx, &gy, &gz)) {
                    Madgwick_Update(gx, gy, gz, ax, ay, az, dt_imu);
                    Compute_Euler_Angles();
                }
            }

            /// ==========================================
            // TASK 2: ĐỌC ENCODER & CHẠY PID (5ms - 200Hz)
            // ==========================================
            if ((now - last_pid_time) >= 15) {
                float dt_pid = (now - last_pid_time) / 1000.0f;
                last_pid_time = now;

                // 1. Tính toán vận tốc ngay tại đây để PID luôn có dữ liệu mới nhất
                Read_Encoder_Delta();
                vL_meas = (delta_left * TICK_TO_M) / dt_pid;
                vR_meas = (delta_right * TICK_TO_M) / dt_pid;

                // Lọc nhiễu vận tốc (Low-pass filter)
                vL_meas = vL_meas * 0.55f + prev_vL_meas * 0.45f;
                vR_meas = vR_meas * 0.55f + prev_vR_meas * 0.45f;
                prev_vL_meas = vL_meas;
                prev_vR_meas = vR_meas;

                // 2. Chạy PID
                float errL = current_vL_cmd - vL_meas;
                float errR = current_vR_cmd - vR_meas;
                float pwmL=0;
                float pwmR=0;

			if(mode==0||mode==1){
	             pwmL = PIDLeft(errL, &prevErrL, &intL, pwml1, dt_pid);
	             pwmR = PIDRight(errR, &prevErrR, &intR, pwmr1, dt_pid);
			}
			if(mode==2){
                 pwmL = PID_FuzLeft(errL, &prevErrL, &intL, pwml1, dt_pid);
                 pwmR = PID_FuzRight(errR, &prevErrR, &intR, pwmr1, dt_pid);

			}
			if(mode==3){
				 pwmL = RBF_PID_Compute(&pid_left, current_vL_cmd, vL_meas);
				 pwmR = RBF_PID_Compute(&pid_right, current_vR_cmd, vR_meas);
            }
			if(mode==4){
				 pwmL = GAPIDLeft(errL, &prevErrL, &intL, pwml1, dt_pid);
				 pwmR = GAPIDRight(errR, &prevErrR, &intR, pwmr1, dt_pid);
			}

            if(mode==5){
            	float forward=pwm_MNR;
            	float turn=pwm_MNL;
            	pwmL=forward+turn;
            	pwmR=forward-turn;
            	float maxVal = max_f(abs(pwmL), abs(pwmR));
            	if (maxVal > 1000.0f) {
            	    pwmL  = pwmL  / maxVal * 1000.0f;
            	    pwmR  = pwmR / maxVal * 1000.0f;
            	}
            }

                pwmL = constrain_f(pwmL, -1000, 1000);
                pwmR = constrain_f(pwmR, -1000, 1000);

                if (fabs(pwmL) < 100) pwmL = 0;
                if (fabs(pwmR) < 100) pwmR = 0;

                // Chống sốc PWM
                pwmL = 0.3f * pwmL_prev + 0.7f * pwmL;
                pwmR = 0.3f * pwmR_prev + 0.7f * pwmR;
                pwmL_prev = pwmL;
                pwmR_prev = pwmR;

                setMotorLeft((int)pwmL);
                setMotorRight((int)pwmR);
            }


            //=========================================
            //TASK 3: Cập nhập ODOMETRY
            // ==========================================
            if ((now - last_odo_time) >= 10) {
                float dt_odo = (now - last_odo_time) / 1000.0f;
                last_odo_time = now;

                // Tính quãng đường đi được trong 10ms (Sử dụng vận tốc từ task PID)
                float dL = vL_meas * dt_odo;
                float dR = vR_meas * dt_odo;
                float dCenter = (dL + dR) / 2.0f;
                float dTheta = (dR - dL) / WHEEL_BASE;

                // Cập nhật Kinematics
                x += dCenter * cosf(theta + dTheta / 2.0f); // Dùng cosf thay vì cos để tận dụng FPU
                y += dCenter * sinf(theta + dTheta / 2.0f);
                theta += dTheta;
                chuan_hoa(&theta);

                // Sensor Fusion (Complementary Filter với IMU)
                if (fabs(theta - imu_yaw) < PI) {
                    theta = theta * 0.4f + imu_yaw * 0.6f;
                } else {
                    if (theta < 0 && imu_yaw > 0) {
                        theta = (theta + 2 * PI) * 0.4f + imu_yaw * 0.6f;
                    } else {
                        theta = theta * 0.4f + (imu_yaw + 2 * PI) * 0.6f;
                    }
                }
                chuan_hoa(&theta);
            }

            // ==========================================
            // TASK 4: GIAO TIẾP UART (30ms - ~33Hz)
            // ==========================================
            if ((now - last_uart_time) >= 30) {
                last_uart_time = now;

                tx_report.header = 0xCCDD;
                tx_report.x = x;
                tx_report.y = y;
                tx_report.theta = theta;
                tx_report.vL_meas = vL_meas;
                tx_report.vR_meas = vR_meas;
            if(mode==3||mode==1||mode==4||mode==0){
                tx_report.kpt = pid_left.Kp;
                tx_report.kit = pid_left.Ki;
                tx_report.kdt = pid_left.Kd;
                tx_report.kpp = pid_right.Kp;
                tx_report.kip = pid_right.Ki;
                tx_report.kdp = pid_right.Kd;
}
             if(mode==2) {
                 tx_report.kpt = kpt_fuz;
                 tx_report.kit = kit_fuz;
                 tx_report.kdt = 15;
                 tx_report.kpp = kpp_fuz;
                 tx_report.kip = kip_fuz;
                 tx_report.kdp = 15;
 }
                tx_report.checksum = calculate_checksum((uint8_t*)&tx_report, sizeof(STM_To_ESP_Packet));

              if (huart1.gState == HAL_UART_STATE_READY) {
                    HAL_UART_Transmit_DMA(&huart1, (uint8_t*)&tx_report, sizeof(STM_To_ESP_Packet));
                }
            }
        }
        /* USER CODE END WHILE */
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE0);

  while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_CSI;
  RCC_OscInitStruct.CSIState = RCC_CSI_ON;
  RCC_OscInitStruct.CSICalibrationValue = RCC_CSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLL1_SOURCE_CSI;
  RCC_OscInitStruct.PLL.PLLM = 1;
  RCC_OscInitStruct.PLL.PLLN = 125;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1_VCIRANGE_2;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1_VCORANGE_WIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_PCLK3;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the programming delay
  */
  __HAL_FLASH_SET_PROGRAM_DELAY(FLASH_PROGRAMMING_DELAY_2);
}

/**
  * @brief GPDMA1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPDMA1_Init(void)
{

  /* USER CODE BEGIN GPDMA1_Init 0 */

  /* USER CODE END GPDMA1_Init 0 */

  /* Peripheral clock enable */
  __HAL_RCC_GPDMA1_CLK_ENABLE();

  /* GPDMA1 interrupt Init */
    HAL_NVIC_SetPriority(GPDMA1_Channel0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel0_IRQn);
    HAL_NVIC_SetPriority(GPDMA1_Channel1_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(GPDMA1_Channel1_IRQn);

  /* USER CODE BEGIN GPDMA1_Init 1 */

  /* USER CODE END GPDMA1_Init 1 */
  /* USER CODE BEGIN GPDMA1_Init 2 */

  /* USER CODE END GPDMA1_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10C043E5;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief ICACHE Initialization Function
  * @param None
  * @retval None
  */
static void MX_ICACHE_Init(void)
{

  /* USER CODE BEGIN ICACHE_Init 0 */

  /* USER CODE END ICACHE_Init 0 */

  /* USER CODE BEGIN ICACHE_Init 1 */

  /* USER CODE END ICACHE_Init 1 */
  /* USER CODE BEGIN ICACHE_Init 2 */

  /* USER CODE END ICACHE_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 4294967295;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 249;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1000;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 4294967295;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 15;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 15;
  if (HAL_TIM_Encoder_Init(&htim5, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * @brief TIM6 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM6_Init(void)
{

  /* USER CODE BEGIN TIM6_Init 0 */

  /* USER CODE END TIM6_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM6_Init 1 */

  /* USER CODE END TIM6_Init 1 */
  htim6.Instance = TIM6;
  htim6.Init.Prescaler = 2499;
  htim6.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim6.Init.Period = 999;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim6) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim6, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM6_Init 2 */

  /* USER CODE END TIM6_Init 2 */

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 921600;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_EnableFifoMode(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, IN1_Pin|IN2_Pin|IN3_Pin|IN4_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : IN1_Pin IN2_Pin IN3_Pin IN4_Pin */
  GPIO_InitStruct.Pin = IN1_Pin|IN2_Pin|IN3_Pin|IN4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

  // Hàm điều khiển bánh trái (Dùng TIM3_CH1 và IN1, IN2)
  void setMotorLeft(int pwm) {
      // Giới hạn dải PWM từ -999 đến 999 (do ARR của TIM3 = 999)
      if (pwm > 999) pwm = 999;
      if (pwm < -999) pwm = -999;

      if (pwm >= 0) {
          // Chạy tới
          HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_RESET);
          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pwm);
      } else {
          // Chạy lùi
          HAL_GPIO_WritePin(IN1_GPIO_Port, IN1_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(IN2_GPIO_Port, IN2_Pin, GPIO_PIN_SET);
          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, -pwm); // Lấy giá trị dương
      }
  }

  // Hàm điều khiển bánh phải (Dùng TIM3_CH2 và IN3, IN4)
  void setMotorRight(int pwm) {
      if (pwm > 999) pwm = 999;
      if (pwm < -999) pwm = -999;

      if (pwm >= 0) {
          // Chạy tới
          HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_RESET);
          HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_SET); // Đảo chiều nếu động cơ bị ngược
          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, pwm);
      } else {
          // Chạy lùi
          HAL_GPIO_WritePin(IN3_GPIO_Port, IN3_Pin, GPIO_PIN_SET);
          HAL_GPIO_WritePin(IN4_GPIO_Port, IN4_Pin, GPIO_PIN_RESET);
          __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, -pwm);
      }
  }


  // Hàm này được thư viện HAL tự động gọi khi DMA nhận đủ dữ liệu
  /* USER CODE BEGIN 4 */
  /* USER CODE BEGIN 4 */
  void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
      if (huart->Instance == USART1) {

          // 1. TIMEOUT RESYNC: Đứt đoạn là Reset SẠCH SẼ toàn bộ biến phụ
          if (HAL_GetTick() - last_rx_time > 50) {
              rx_state = WAIT_HEADER_1;
              current_packet_type = PACKET_NONE; // Reset
              expected_length = 0;               // Reset
              rx_index = 0;
          }
          last_rx_time = HAL_GetTick();

          // --- BẮT ĐẦU STATE MACHINE (Memoryless) ---
          switch (rx_state) {
              case WAIT_HEADER_1:
                  // Chỉ lưu byte, chưa vội phán xét loại gói tin
                  if (rx_byte_tmp == 0xBB || rx_byte_tmp == 0xFF) {
                      rx_buffer[0] = rx_byte_tmp;
                      rx_state = WAIT_HEADER_2;
                  }
                  break;

              case WAIT_HEADER_2:
                  rx_buffer[1] = rx_byte_tmp;

                  // Chờ đủ 2 byte mới bắt đầu chốt hạ Packet Type
                  if (rx_buffer[0] == 0xBB && rx_byte_tmp == 0xAA) {
                      current_packet_type = PACKET_REALTIME;
                      expected_length = sizeof(ESP_To_STM_Packet); // = 11
                      rx_index = 2;
                      rx_state = READ_PAYLOAD;
                  }
                  else if (rx_buffer[0] == 0xFF && rx_byte_tmp == 0xEE) {
                      current_packet_type = PACKET_CONFIG;
                      expected_length = sizeof(ESP_To_STM_Config); // = 19
                      rx_index = 2;
                      rx_state = READ_PAYLOAD;
                  }
                  else if (rx_byte_tmp == 0xBB || rx_byte_tmp == 0xFF) {
                      // Kỹ thuật "Trượt Header" siêu việt: Giữ lại byte hiện tại làm byte 0
                      rx_buffer[0] = rx_byte_tmp;
                      rx_state = WAIT_HEADER_2;
                  }
                  else {
                      // Tạch header -> Xóa sổ mọi thứ
                      rx_state = WAIT_HEADER_1;
                  }
                  break;

              case READ_PAYLOAD:
                  rx_buffer[rx_index++] = rx_byte_tmp;

                  // Dùng expected_length để linh hoạt điểm dừng
                  if (rx_index >= expected_length) {
                      uint8_t calc_chk = calculate_checksum(rx_buffer, expected_length);

                      if (calc_chk == rx_buffer[expected_length - 1]) {

                          // RẼ NHÁNH XỬ LÝ (Sử dụng memcpy chống HardFault)
                          if (current_packet_type == PACKET_REALTIME) {
                              ESP_To_STM_Packet cmd;
                              memcpy(&cmd, rx_buffer, sizeof(cmd)); // An toàn tuyệt đối

                              current_vL_cmd = cmd.vL_cmd;
                              current_vR_cmd = cmd.vR_cmd;
                          }
                          else if (current_packet_type == PACKET_CONFIG) {
                              ESP_To_STM_Config cfg;
                              memcpy(&cfg, rx_buffer, sizeof(cfg));

                              // Cập nhật biến cấu hình
                              pwm_MNL = cfg.param1;
                              pwm_MNR = cfg.param2;
                              mode = cfg.mode_int;
                              current_vL_cmd = 0.0f;
                              current_vR_cmd = 0.0f;
                              intL = 0;
                              intR = 0;
                          }
                      }

                      // 🎯 RESET FULL STATE SAU KHI XONG 1 GÓI (Hoặc sai Checksum)
                      rx_state = WAIT_HEADER_1;
                      current_packet_type = PACKET_NONE;
                      expected_length = 0;
                  }
                  break;
          }

          // LUÔN MỒI LẠI DMA 1 BYTE
          HAL_UART_Receive_DMA(&huart1, &rx_byte_tmp, 1);
      }
  }
  /* USER CODE END 4 */
  /* USER CODE END 4 */
  void Read_Encoder_Delta() {
      // 1. Đọc trực tiếp thanh ghi đếm (Tốc độ ánh sáng, tốn 1 chu kỳ máy)
      current_left_ticks = TIM5->CNT;
      current_right_ticks = TIM2->CNT;

      // 2. Tính toán chênh lệch xung (Delta Ticks)
      // Phép trừ 2 số unsigned 32-bit sau đó ép kiểu sang signed 32-bit
      // sẽ tự động giải quyết mượt mà sự cố Tràn (Overflow / Underflow).
      delta_left = (int32_t)(current_left_ticks - prev_left_ticks);
      delta_right = (int32_t)(current_right_ticks - prev_right_ticks);

      // Xử lý đảo chiều bánh phải (Vì 2 động cơ lắp ngược hướng nhau trên khung xe)
      // Tùy theo cách đấu dây A/B mà bạn có thể cần thêm dấu âm ở delta_right hoặc delta_left
      delta_right = -delta_right;

      // 3. Cập nhật lại giá trị cũ
      prev_left_ticks = current_left_ticks;
      prev_right_ticks = current_right_ticks;
  }


  // Khởi động MPU6050
  void MPU6050_Init(void) {
      uint8_t check;
      uint8_t Data = 0;

      // Kiểm tra xem MPU6050 có phản hồi không (Đọc thanh ghi WHO_AM_I)
      HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x75, 1, &check, 1, 100);
      if (check == 0x68) {
          // Đánh thức cảm biến (Ghi 0 vào thanh ghi PWR_MGMT_1)
          Data = 0x00;
          HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x6B, 1, &Data, 1, 100);

          // Cấu hình Gyroscope (±250°/s) - Thanh ghi GYRO_CONFIG
          Data = 0x00;
          HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x1B, 1, &Data, 1, 100);

          // Cấu hình Accelerometer (±2g) - Thanh ghi ACCEL_CONFIG
          Data = 0x00;
          HAL_I2C_Mem_Write(&hi2c1, MPU_ADDR, 0x1C, 1, &Data, 1, 100);
      }
  }

  // Đọc dữ liệu thô và chuyển đổi (Sử dụng con trỏ thay cho tham chiếu & của C++)
  bool readIMU(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) {
      uint8_t buf[14];

      // Đọc liên tiếp 14 bytes từ thanh ghi ACCEL_XOUT_H (0x3B)
      if (HAL_I2C_Mem_Read(&hi2c1, MPU_ADDR, 0x3B, 1, buf, 14, 100) != HAL_OK) {
          return false;
      }

      int16_t rawAx = (buf[0] << 8) | buf[1];
      int16_t rawAy = (buf[2] << 8) | buf[3];
      int16_t rawAz = (buf[4] << 8) | buf[5];
      // buf[6], buf[7] là nhiệt độ (bỏ qua)
      int16_t rawGx = (buf[8] << 8) | buf[9];
      int16_t rawGy = (buf[10] << 8) | buf[11];
      int16_t rawGz = (buf[12] << 8) | buf[13];

      const float A_SCALE = 9.80665f / 16384.0f;
      const float G_SCALE = 1.0f / 131.0f;

      *ax = rawAx * A_SCALE;
      *ay = rawAy * A_SCALE;
      *az = rawAz * A_SCALE;

      *gx = (rawGx * G_SCALE * DEG2RAD) - gyroBiasX;
      *gy = (rawGy * G_SCALE * DEG2RAD) - gyroBiasY;
      *gz = (rawGz * G_SCALE * DEG2RAD) - gyroBiasZ;

      return true;
  }

  // Hàm lấy Offset (Calibrate)
  void calibrateGyroBias(int samples) {
      float sumX = 0.0f, sumY = 0.0f, sumZ = 0.0f;
      int valid = 0;

      for (int i = 0; i < samples; i++) {
          float ax, ay, az, gx, gy, gz;

          // Không trừ Bias khi đang đo Bias
          float tempBiasX = gyroBiasX, tempBiasY = gyroBiasY, tempBiasZ = gyroBiasZ;
          gyroBiasX = 0; gyroBiasY = 0; gyroBiasZ = 0;

          if (readIMU(&ax, &ay, &az, &gx, &gy, &gz)) {
              sumX += gx; // gx lúc này chưa bị trừ bias
              sumY += gy;
              sumZ += gz;
              valid++;
          }

          gyroBiasX = tempBiasX; gyroBiasY = tempBiasY; gyroBiasZ = tempBiasZ;
          HAL_Delay(5);
      }

      if (valid > 0) {
          gyroBiasX = sumX / (float)valid;
          gyroBiasY = sumY / (float)valid;
          gyroBiasZ = sumZ / (float)valid;
      }
  }

  // Thuật toán Madgwick (Đã sử dụng hàm có hậu tố 'f' để kích hoạt FPU phần cứng)
  void Madgwick_Update(float gx, float gy, float gz, float ax, float ay, float az, float dt) {
      if (dt <= 0.0f) return;

      float norm = sqrtf(ax*ax + ay*ay + az*az);
      if (norm == 0.0f) return;
      ax /= norm; ay /= norm; az /= norm;

      float q0q0 = q0*q0, q1q1 = q1*q1, q2q2 = q2*q2, q3q3 = q3*q3;
      float _2q0 = 2.0f*q0, _2q1 = 2.0f*q1, _2q2 = 2.0f*q2, _2q3 = 2.0f*q3;
      float _4q1 = 4.0f*q1, _4q2 = 4.0f*q2;
      float q0q1 = q0*q1, q0q2 = q0*q2, q1q3 = q1*q3, q2q3 = q2*q3;

      float f1 = 2.0f*(q1q3 - q0q2) - ax;
      float f2 = 2.0f*(q0q1 + q2q3) - ay;
      float f3 = 2.0f*(0.5f - q1q1 - q2q2) - az;

      float s0 = -_2q2 * f1 + _2q1 * f2;
      float s1 =  _2q3 * f1 + _2q0 * f2 - _4q1 * f3;
      float s2 = -_2q0 * f1 + _2q3 * f2 - _4q2 * f3;
      float s3 =  _2q1 * f1 + _2q2 * f2;

      float sNorm = sqrtf(s0*s0 + s1*s1 + s2*s2 + s3*s3);
      if (sNorm == 0.0f) return;
      s0 /= sNorm; s1 /= sNorm; s2 /= sNorm; s3 /= sNorm;

      float qDot0 = 0.5f * (-q1*gx - q2*gy - q3*gz) - beta_madgwick * s0;
      float qDot1 = 0.5f * ( q0*gx + q2*gz - q3*gy) - beta_madgwick * s1;
      float qDot2 = 0.5f * ( q0*gy - q1*gz + q3*gx) - beta_madgwick * s2;
      float qDot3 = 0.5f * ( q0*gz + q1*gy - q2*gx) - beta_madgwick * s3;

      q0 += qDot0 * dt; q1 += qDot1 * dt;
      q2 += qDot2 * dt; q3 += qDot3 * dt;

      float qNorm = sqrtf(q0*q0 + q1*q1 + q2*q2 + q3*q3);
      if (qNorm == 0.0f) {
          q0 = 1.0f; q1 = 0.0f; q2 = 0.0f; q3 = 0.0f;
          return;
      }
      q0 /= qNorm; q1 /= qNorm; q2 /= qNorm; q3 /= qNorm;
  }

  // Chuyển đổi Quaternion sang góc Euler
  void Compute_Euler_Angles(void) {
      imu_yaw = atan2f(2.0f*(q0*q3 + q1*q2), 1.0f - 2.0f*(q2*q2 + q3*q3));

      float v = 2.0f*(q0*q2 - q3*q1);
      if (v > 1.0f) v = 1.0f;
      if (v < -1.0f) v = -1.0f;
      imu_pitch = asinf(v);

      imu_roll = atan2f(2.0f*(q0*q1 + q2*q3), 1.0f - 2.0f*(q1*q1 + q2*q2));
  }





  // 4. Hàm liên thuộc hình tam giác

  // 1. Hàm liên thuộc hình chuông Gauss (Chạy siêu nhanh nhờ FPU)
    float fuzzy_gaussmf(float x, float c, float sigma) {
        if (sigma == 0.0f) return (x == c) ? 1.0f : 0.0f; // Tránh lỗi chia cho 0

        float tmp = (x - c) / sigma;
        return expf(-0.5f * tmp * tmp); // Hàm e^(-0.5 * ((x-c)/sigma)^2)
    }

    // 2. Hàm tính toán Kp, Ki tự động (Phiên bản Fully Gaussian)
    void computeFuzzyPI(float e, float de, float *Kp_fuzzy, float *Ki_fuzzy) {
        // Chuẩn hóa đầu vào về dải [-1, 1]
        float e_norm = constrain_f(e / MAX_E, -1.0f, 1.0f);
        float de_norm = constrain_f(de / MAX_DE, -1.0f, 1.0f);

        // Mờ hóa: 5 tập (NB, NS, ZE, PS, PB)
        float mu_e[5], mu_de[5];

        // Tọa độ MFs chuẩn Gauss: {Center (c), Sigma (độ choãi)}
        // Dùng chung một độ choãi 0.25 cho tất cả để tạo sự cân bằng tuyệt đối
        const float params[5][2] = {
            {-1.0f, 0.25f}, // NB: Đỉnh tại -1.0
            {-0.5f, 0.25f}, // NS: Đỉnh tại -0.5
            { 0.0f, 0.25f}, // ZE: Đỉnh tại 0.0
            { 0.5f, 0.25f}, // PS: Đỉnh tại 0.5
            { 1.0f, 0.25f}  // PB: Đỉnh tại 1.0
        };

        // Tính độ phụ thuộc (Degree of membership) bằng đồ thị Gauss
        for (int i = 0; i < 5; i++) {
            mu_e[i]  = fuzzy_gaussmf(e_norm,  params[i][0], params[i][1]);
            mu_de[i] = fuzzy_gaussmf(de_norm, params[i][0], params[i][1]);
        }

        // Suy diễn và Giải mờ (Singleton Sugeno)
        float num_Kp = 0, num_Ki = 0, den = 0;

        for (int i = 0; i < 5; i++) {       // Lặp qua e
            for (int j = 0; j < 5; j++) {     // Lặp qua de
                // Luật AND (MIN)
                float weight = fmin(mu_e[i], mu_de[j]);

                // Khác với tam giác (có chỗ bằng 0), đồ thị Gauss luôn > 0 (dù rất nhỏ)
                // Ta đặt ngưỡng 0.01 để bỏ qua các giá trị quá bé, tiết kiệm chu kỳ máy
                if (weight > 0.01f) {
                    int Kp_idx = rule_Kp[i][j];
                    int Ki_idx = rule_Ki[i][j];

                    num_Kp += weight * Kp_out[Kp_idx];
                    num_Ki += weight * Ki_out[Ki_idx];
                    den += weight;
                }
            }
        }

        // Trả kết quả
        if (den > 0) {
            *Kp_fuzzy = num_Kp / den;
            *Ki_fuzzy = num_Ki / den;
        } else {
            *Kp_fuzzy = 0;
            *Ki_fuzzy = 0;
        }
    }
/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM7 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM7)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

