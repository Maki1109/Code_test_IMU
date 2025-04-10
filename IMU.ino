#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu;

#define ALPHA 0.97  // Hệ số bộ lọc bổ sung
float angleX = 0;   // Giá trị góc nghiêng hiện tại

unsigned long lastTime = 0;  // Thời điểm đọc dữ liệu trước đó

void setup() {
    Serial.begin(115200);
    Wire.begin();
    mpu.initialize();
    lastTime = millis();  // Lưu lại thời gian bắt đầu
}

// Đọc dữ liệu từ MPU6050
void getIMUData(float &ax, float &az, float &gx) {
    int16_t rawAx, rawAz, rawGx;
    mpu.getMotion6(&rawAx, nullptr, &rawAz, &rawGx, nullptr, nullptr);

    ax = rawAx / 16384.0;  // Chuyển đổi gia tốc từ raw sang g-force
    az = rawAz / 16384.0;
    gx = rawGx / 131.0;     // Chuyển đổi vận tốc góc từ raw sang °/s
}

// Bộ lọc bổ sung (Complementary Filter)
float complementaryFilter(float accelAngle, float gyroRate, float dt) {
    return ALPHA * (angleX + gyroRate * dt) + (1 - ALPHA) * accelAngle;
}

void loop() {
    float ax, az, gx;
    getIMUData(ax, az, gx);

    // Tính toán góc từ gia tốc kế (Acc)
    float accelAngleX = atan2(ax, az) * 180.0 / PI;

    // Tính khoảng thời gian giữa 2 lần đo (Δt)
    unsigned long currentTime = millis();
    float dt = (currentTime - lastTime) / 1000.0;  // Đổi ms sang giây
    lastTime = currentTime;

    // Kết hợp dữ liệu từ gia tốc kế và con quay hồi chuyển
    angleX = complementaryFilter(accelAngleX, gx, dt);

    // Hiển thị dữ liệu trên Serial Monitor
    Serial.print("Raw Angle X: ");
    Serial.println(accelAngleX);
    Serial.print("Filtered Angle X: ");
    Serial.println(angleX);

    delay(10);  // Giảm trễ để cập nhật nhanh hơn
}
