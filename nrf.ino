//  SỬA ĐỔI cho FC ESP32: nhận lệnh từ tay điều khiển nRF24L01 thay cho SBUS.
//  File này THAY THẾ rc_sbus.ino (xoá rc_sbus.ino khỏi thư mục sketch).
//
//  Giữ nguyên giao diện mà phần còn lại của firmware đang dùng:
//     void rc_init();
//     bool rc_update();                 // true = còn sóng
//     int  rc_get_channel(int index);   // 1..16, đơn vị us như SBUS (1000..2000)
//  nên control_task không phải sửa.
//
//  Tay điều khiển chỉ có 4 trục và 2 nút, không có công tắc 3 nấc, nên mô phỏng:
//     CH_ROLL / CH_PITCH / CH_THROTTLE / CH_YAW  <- 4 trục của TX
//     CH_ARM  : nút TRÁI bấm 1 lần = bật arm (chỉ khi ga thấp), bấm lần nữa = disarm
//     CH_MODE : nút PHẢI bấm 1 lần = đổi qua lại ANGLE <-> ALT HOLD
//  (công tắc ảo chỉ nhận giá trị 1000 hoặc 2000)
//
//  An toàn: bấm nút TRÁI khi đang bay là disarm NGAY. Mất sóng quá NRF_TIMEOUT_MS
//  thì rc_update() trả false, công tắc ảo trả về disarm + chế độ ANGLE.
//
//  nRF24 dùng bus HSPI riêng (chân 16/34/13/17), tách khỏi VSPI của ICM20602.
// ============================================================================

#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>

// Struct PHẢI giống hệt bên TX (4 int + 2 bool)
struct NrfControlData {
  int  pwmThrottle;
  int  pwmYaw;
  int  pwmPitch;
  int  pwmRoll;
  bool btnLeft;
  bool btnRight;
};

#define NRF_CHANNEL          76
#define NRF_TIMEOUT_MS      250      // quá thời gian này không có gói mới = mất sóng
#define NRF_BTN_ACTIVE_LOW  true     // nút nối INPUT_PULLUP: bấm = LOW = false
#define NRF_ARM_THR_MAX    1050      // chỉ cho bật arm khi ga dưới mức này
#define NRF_LOCKOUT_MS      300      // chống dội nút

static SPIClass nrf_spi(HSPI);
static RF24     nrf_radio(PIN_NRF_CE, PIN_NRF_CSN);
static const byte nrf_address[6] = "DRONE";

static NrfControlData nrf_data;
static int  nrf_ch[17];
static bool nrf_present   = false;
static bool nrf_link_seen = false;
static unsigned long nrf_last_rx_ms = 0;

// Công tắc ảo
static bool nrf_arm_req  = false;
static bool nrf_mode_alt = false;
static bool nrf_prev_left = false, nrf_prev_right = false;
static unsigned long nrf_last_left_ms = 0, nrf_last_right_ms = 0;


static void nrf_set_neutral() {
  for (int i = 1; i <= 16; i++) nrf_ch[i] = 1000;
  nrf_ch[CH_ROLL]     = 1500;
  nrf_ch[CH_PITCH]    = 1500;
  nrf_ch[CH_YAW]      = 1500;
  nrf_ch[CH_THROTTLE] = 1000;
}

// Đảo chiều quanh tâm 1500 (inv = 1: giữ nguyên, -1: đảo)
static int nrf_axis(int us, int inv) {
  us = constrain(us, 1000, 2000);
  return 1500 + inv * (us - 1500);
}

static bool nrf_pressed(bool v) { return NRF_BTN_ACTIVE_LOW ? !v : v; }

static bool nrf_data_valid() {
  return nrf_data.pwmThrottle >= 900 && nrf_data.pwmThrottle <= 2100 &&
         nrf_data.pwmYaw      >= 900 && nrf_data.pwmYaw      <= 2100 &&
         nrf_data.pwmPitch    >= 900 && nrf_data.pwmPitch    <= 2100 &&
         nrf_data.pwmRoll     >= 900 && nrf_data.pwmRoll     <= 2100;
}


void rc_init() {
  nrf_set_neutral();

  nrf_spi.begin(PIN_NRF_SCK, PIN_NRF_MISO, PIN_NRF_MOSI, PIN_NRF_CSN);
  nrf_present = nrf_radio.begin(&nrf_spi);

  if (!nrf_present) {
    Serial.println("NRF24 CHIP NOT CONNECTED - se khong arm duoc");
    return;
  }
  Serial.println("NRF24 CHIP CONNECTED");

  nrf_radio.setChannel(NRF_CHANNEL);
  nrf_radio.setDataRate(RF24_1MBPS);
  nrf_radio.setPALevel(RF24_PA_LOW);
  nrf_radio.setCRCLength(RF24_CRC_16);
  nrf_radio.openReadingPipe(0, nrf_address);
  nrf_radio.startListening();
}


// Xử lý một gói hợp lệ: cập nhật trục và công tắc ảo
static void nrf_handle_packet() {
  unsigned long now = millis();
  nrf_last_rx_ms = now;

  bool left  = nrf_pressed(nrf_data.btnLeft);
  bool right = nrf_pressed(nrf_data.btnRight);

  // Gói đầu tiên sau khi có sóng: ghi nhớ trạng thái nút, không tính là một lần bấm.
  // Nhờ vậy nút đang bị giữ lúc bật nguồn không làm arm / đổi chế độ.
  if (!nrf_link_seen) {
    nrf_link_seen  = true;
    nrf_prev_left  = left;
    nrf_prev_right = right;
    nrf_last_left_ms = nrf_last_right_ms = now;
  }

  // Nút TRÁI: bật/tắt arm
  if (left && !nrf_prev_left && (now - nrf_last_left_ms) > NRF_LOCKOUT_MS) {
    nrf_last_left_ms = now;
    if (nrf_arm_req)                                         nrf_arm_req = false;   // disarm ngay
    else if (nrf_data.pwmThrottle < NRF_ARM_THR_MAX)         nrf_arm_req = true;
  }

  // Nút PHẢI: đổi chế độ
  if (right && !nrf_prev_right && (now - nrf_last_right_ms) > NRF_LOCKOUT_MS) {
    nrf_last_right_ms = now;
    nrf_mode_alt = !nrf_mode_alt;
  }

  nrf_prev_left  = left;
  nrf_prev_right = right;

  nrf_ch[CH_THROTTLE] = nrf_axis(nrf_data.pwmThrottle, RC_INV_THROTTLE);
  nrf_ch[CH_ROLL]     = nrf_axis(nrf_data.pwmRoll,     RC_INV_ROLL);
  nrf_ch[CH_PITCH]    = nrf_axis(nrf_data.pwmPitch,    RC_INV_PITCH);
  nrf_ch[CH_YAW]      = nrf_axis(nrf_data.pwmYaw,      RC_INV_YAW);
  nrf_ch[CH_ARM]      = nrf_arm_req  ? 2000 : 1000;
  nrf_ch[CH_MODE]     = nrf_mode_alt ? 2000 : 1000;
}


// Trả về true khi vẫn còn sóng, false khi mất sóng.
bool rc_update() {
  if (nrf_present) {
    bool got = false;
    while (nrf_radio.available()) {            // lấy hết FIFO, giữ gói mới nhất
      nrf_radio.read(&nrf_data, sizeof(nrf_data));
      got = true;
    }
    if (got && nrf_data_valid()) nrf_handle_packet();
  }

  bool ok = nrf_link_seen && ((millis() - nrf_last_rx_ms) <= NRF_TIMEOUT_MS);

  if (!ok && nrf_link_seen) {                  // vừa mất sóng: về trạng thái an toàn
    nrf_link_seen = false;
    nrf_arm_req   = false;
    nrf_mode_alt  = false;
    nrf_set_neutral();
  }
  return ok;
}


int rc_get_channel(int index) {
  if (index < 1 || index > 16) return 1000;
  return nrf_ch[index];
}
