
#include <Arduino.h>
#include <Wire.h>
#include <math.h>

const int MPU_ADDR = 0x68;

float angleX = 0;
float angleY = 0;

float smoothX = 0;
float smoothY = 0;

void setup() {
    Serial.begin(9600);
    Wire.begin();

    // Wake up MPU6050
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);

    delay(1000);
}

void loop() {
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);

    if (Wire.requestFrom(MPU_ADDR, 6, true) != 6) {
        delay(20);
        return;
    }

    int16_t AcX = (Wire.read() << 8) | Wire.read();
    int16_t AcY = (Wire.read() << 8) | Wire.read();
    int16_t AcZ = (Wire.read() << 8) | Wire.read();

    // Convert acceleration into tilt angles
    angleX = atan2(
        (float)AcY,
        sqrt((float)AcX * AcX + (float)AcZ * AcZ)
    ) * 180.0 / PI;

    angleY = atan2(
        -(float)AcX,
        sqrt((float)AcY * AcY + (float)AcZ * AcZ)
    ) * 180.0 / PI;

    // Smooth sensor readings
    smoothX = smoothX * 0.85 + angleX * 0.15;
    smoothY = smoothY * 0.85 + angleY * 0.15;

    // Send data to Python
    Serial.print(smoothX, 2);
    Serial.print(",");
    Serial.println(smoothY, 2);

    delay(20);
}
