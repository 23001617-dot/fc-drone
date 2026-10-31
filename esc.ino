
//  SỬA ĐỔI cho FC ESP32 dùng ESC BLHeli_S (BLS J_H_30 REV16.7): thay PWM bằng DShot300.
//
//  Vì sao đổi: ESC này không nhận PWM 391Hz, chỉ chạy được với DShot.
//
//  Cách hoạt động
//    - control_task vẫn gọi esc_write(m1..m4) với giá trị 800..1600 như bản gốc,
//      nên KHÔNG phải sửa mixer hay PID. Hàm này đổi 800..1600 sang DShot 48..DSHOT_MAX_OUT.
//    - Việc phát khung DShot do một task riêng (core 0) lo, mỗi 1 ms một khung,
//      nên ESC luôn có tín hiệu, kể cả lúc setup() đang hiệu chỉnh gyro.
//    - Task này tự gửi ga 0 nếu control_task ngừng gọi esc_write quá ESC_STALE_MS.
//
//  Giá trị đầu vào:  <= ESC_IDLE  -> DShot 0 (motor dừng)
//                    ESC_IDLE..ESC_MAX -> DShot DSHOT_MIN_THROTTLE..DSHOT_MAX_OUT
// ============================================================================

#include "soc/gpio_struct.h"

static const int esc_pins[4] = {PIN_ESC_1, PIN_ESC_2, PIN_ESC_3, PIN_ESC_4};

// control_task ghi, esc_dshot_task đọc
static volatile uint16_t esc_dshot_val[4] = {0, 0, 0, 0};
static volatile uint32_t esc_last_write_ms = 0;


static inline void IRAM_ATTR esc_pin_high(int p) {
  if (p < 32) GPIO.out_w1ts = (1UL << p);
  else        GPIO.out1_w1ts.val = (1UL << (p - 32));
}

static inline void IRAM_ATTR esc_pin_low(int p) {
  if (p < 32) GPIO.out_w1tc = (1UL << p);
  else        GPIO.out1_w1tc.val = (1UL << (p - 32));
}


// Phát một khung DShot300 (16 bit) lên một chân. Tắt ngắt trong ~53 us.
static void IRAM_ATTR esc_dshot_send(int pin, uint16_t value) {
  uint16_t packet = (value << 1);                       // bit telemetry = 0
  uint8_t crc = (packet ^ (packet >> 4) ^ (packet >> 8)) & 0x0F;
  packet = (packet << 4) | crc;

  uint32_t mhz    = getCpuFrequencyMhz();
  uint32_t period = mhz * 10 / 3;   // 3.33 us
  uint32_t t1h    = mhz * 5 / 2;    // 2.50 us
  uint32_t t0h    = mhz * 5 / 4;    // 1.25 us

  noInterrupts();
  uint32_t start = ESP.getCycleCount();
  for (int i = 15; i >= 0; i--) {
    uint32_t hi = (packet & (1 << i)) ? t1h : t0h;
    esc_pin_high(pin);
    while ((ESP.getCycleCount() - start) < hi);
    esc_pin_low(pin);
    while ((ESP.getCycleCount() - start) < period);
    start += period;
  }
  interrupts();
}


// 800..1600 (đơn vị của bản gốc) -> DShot
static uint16_t esc_to_dshot(int v) {
  if (v <= ESC_IDLE) return 0;
  long d = DSHOT_MIN_THROTTLE +
           (long)(v - ESC_IDLE) * (DSHOT_MAX_OUT - DSHOT_MIN_THROTTLE) / (ESC_MAX - ESC_IDLE);
  if (d < DSHOT_MIN_THROTTLE) d = DSHOT_MIN_THROTTLE;
  if (d > DSHOT_MAX_OUT)      d = DSHOT_MAX_OUT;
  return (uint16_t)d;
}


// Phát DShot 1 kHz, độc lập với vòng điều khiển 500 Hz.
static void esc_dshot_task(void *parameter) {
  TickType_t wake = xTaskGetTickCount();
  for (;;) {
    bool stale = ((uint32_t)millis() - esc_last_write_ms) > ESC_STALE_MS;
    for (int i = 0; i < 4; i++) {
      esc_dshot_send(esc_pins[i], stale ? 0 : esc_dshot_val[i]);
    }
    vTaskDelayUntil(&wake, pdMS_TO_TICKS(1));
  }
}


void esc_init() {
  for (int i = 0; i < 4; i++) {
    pinMode(esc_pins[i], OUTPUT);
    digitalWrite(esc_pins[i], LOW);
    esc_dshot_val[i] = 0;
  }
  esc_last_write_ms = millis();

  // Priority 6, core 0: cao hơn rc_task (4) và baro_task (2)
  xTaskCreatePinnedToCore(esc_dshot_task, "DSHOT", 3072, NULL, 6, NULL, 0);
}


void esc_write(int m1, int m2, int m3, int m4) {
  esc_dshot_val[0] = esc_to_dshot(m1);
  esc_dshot_val[1] = esc_to_dshot(m2);
  esc_dshot_val[2] = esc_to_dshot(m3);
  esc_dshot_val[3] = esc_to_dshot(m4);
  esc_last_write_ms = millis();
}
