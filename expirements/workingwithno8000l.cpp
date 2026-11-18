
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

// Components
Adafruit_MPU6050 mpu;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// Buzzer pins
const int buzzer1 = 8;
const int buzzer2 = 9;

// Shaking sensitivity (m/s^2)
const float LIGHT_THRESHOLD = 1.5;
const float HEAVY_THRESHOLD = 4.0;

// Filter and timing
const float FILTER_ALPHA = 0.20;
const unsigned long HOLD_TIME = 1500;
const unsigned long SAMPLE_INTERVAL = 20;

// Previous acceleration readings
float prevX = 0;
float prevY = 0;
float prevZ = 0;
float shaking = 0;

bool firstReading = true;
int alertLevel = 0;
unsigned long lastShakeTime = 0;
unsigned long lastSampleTime = 0;

void showStatus(int level) {
    lcd.clear();

    if (level == 0) {
        lcd.setCursor(0, 0);
        lcd.print("System Normal");
        lcd.setCursor(0, 1);
        lcd.print("Monitoring...");
    }
    else if (level == 1) {
        lcd.setCursor(0, 0);
        lcd.print("Shaking detected");
        lcd.setCursor(0, 1);
        lcd.print("Stay Alert!");
    }
    else {
        lcd.setCursor(0, 0);
        lcd.print("Heavy Shaking");
        lcd.setCursor(0, 1);
        lcd.print("Evacuate Now!");
    }
}

void setup() {
    Serial.begin(9600);

    pinMode(buzzer1, OUTPUT);
    pinMode(buzzer2, OUTPUT);

    noTone(buzzer1);
    noTone(buzzer2);

    Wire.begin();

    lcd.init();
    lcd.backlight();

    lcd.setCursor(0, 0);
    lcd.print("Earthquake Alert");
    lcd.setCursor(0, 1);
    lcd.print("Starting...");

    if (!mpu.begin()) {
        lcd.clear();
        lcd.print("MPU6050 ERROR");

        Serial.println("MPU6050 not found!");

        while (true) {
            delay(100);
        }
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    delay(1000);
    showStatus(0);
}

void loop() {
    unsigned long now = millis();

    if (now - lastSampleTime < SAMPLE_INTERVAL) {
        return;
    }
    lastSampleTime = now;

    sensors_event_t a, g, temp;
    mpu.getEvent(&a, &g, &temp);

    float x = a.acceleration.x;
    float y = a.acceleration.y;
    float z = a.acceleration.z;

    if (firstReading) {
        prevX = x;
        prevY = y;
        prevZ = z;
        firstReading = false;
        return;
    }

    // Measure acceleration changes
    float dx = x - prevX;
    float dy = y - prevY;
    float dz = z - prevZ;

    float rawShaking = sqrt(
        dx * dx +
        dy * dy +
        dz * dz
    );

    // Smooth the measurement
    shaking = FILTER_ALPHA * rawShaking +
              (1.0 - FILTER_ALPHA) * shaking;

    prevX = x;
    prevY = y;
    prevZ = z;

    // Determine shaking level
    int detectedLevel = 0;

    if (shaking >= HEAVY_THRESHOLD) {
        detectedLevel = 2;
    }
    else if (shaking >= LIGHT_THRESHOLD) {
        detectedLevel = 1;
    }

    // Keep alerts visible briefly
    if (detectedLevel > 0) {
        lastShakeTime = now;
    }

    if (detectedLevel > alertLevel) {
        alertLevel = detectedLevel;
        showStatus(alertLevel);
    }
    else if (detectedLevel < alertLevel &&
             now - lastShakeTime >= HOLD_TIME) {
        alertLevel = detectedLevel;
        showStatus(alertLevel);
    }

    // Buzzer behavior
    if (alertLevel == 0) {
        noTone(buzzer1);
        noTone(buzzer2);
    }
    else if (alertLevel == 1) {
        // One buzzer sounds
        tone(buzzer1, 1000);
        noTone(buzzer2);
    }
    else {
        // Both buzzers sound
        // Uno tone() supports one frequency at a time.
        // Use the same frequency for both passive buzzers
        // by driving both pins together.
        noTone(buzzer1);
        noTone(buzzer2);

        bool beepOn = (now / 200) % 2 == 0;
        digitalWrite(buzzer1, beepOn ? HIGH : LOW);
        digitalWrite(buzzer2, beepOn ? HIGH : LOW);
    }

    Serial.print("Shaking: ");
    Serial.print(shaking);
    Serial.print(" | Alert: ");
    Serial.println(alertLevel);
}
