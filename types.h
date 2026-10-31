
#ifndef KYTHUATUAV_TYPES_H
#define KYTHUATUAV_TYPES_H

// Các kiểu dữ liệu dùng chung phải nằm trong header, không để trong file .ino.
// Lý do: Arduino tự sinh prototype cho mọi hàm rồi chèn lên đầu sketch. Nếu
// một hàm có tham số kiểu struct mà struct đó khai báo trong .ino, prototype
// sẽ xuất hiện trước phần khai báo struct và trình biên dịch báo lỗi
// "was not declared in this scope". Đặt trong .h thì #include chạy trước,
// nên kiểu luôn được biết trước prototype.

// Trạng thái nội bộ của một bộ điều khiển PID
struct PidState {
  float prev_error;
  float integral;
};

// Cảm biến nào có mặt lúc khởi động
struct SensorPresent {
  bool imu;
  bool baro;
};

#endif
