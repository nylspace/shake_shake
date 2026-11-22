#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>
#include <string.h>

Adafruit_MPU6050 mpu;
LiquidCrystal_I2C lcd(0x27, 16, 2);
SoftwareSerial sim800(10, 11);

// Replace with your phone number
const char PHONE_NUMBER[] = "+639XXXXXXXXX";

// One passive buzzer
const byte BUZZER_PIN = 8;

// Single shaking threshold (m/s^2)
const float SHAKE_THRESHOLD = 1.5;
const float FILTER_ALPHA = 0.20;

// Timing (ms)
const unsigned long SAMPLE_INTERVAL = 20;
const unsigned long HOLD_TIME = 1500;     // keep alert on after shaking stops
const unsigned long SHAKE_GAP = 500;      // gap that ends a shaking event
const unsigned long PRINT_INTERVAL = 1000;

// Sensor data
float prevX = 0, prevY = 0, prevZ = 0;
float shaking = 0;
float rawShaking = 0;
bool firstReading = true;

// Alert state (single level: on / off)
bool alertActive = false;
bool shakeEventActive = false;

unsigned long lastSampleTime = 0;
unsigned long lastShakeTime = 0;
unsigned long lastPrintTime = 0;

// Research statistics
unsigned long eventCount = 0;
unsigned long smsSuccessCount = 0;
unsigned long smsFailureCount = 0;
float peakShaking = 0;

// SMS (one message type, sent once per shaking event)
enum SMSState {
    SMS_IDLE,
    SMS_WAIT_TEXT_MODE,
    SMS_WAIT_PROMPT,
    SMS_WAIT_RESULT
};

SMSState smsState = SMS_IDLE;

bool smsPending = false;
unsigned long smsStepTime = 0;
unsigned long smsStartTime = 0;
unsigned long detectionTime = 0;
unsigned long smsRetryAfter = 0;
byte smsRetryCount = 0;

const unsigned long SMS_RETRY_DELAY = 5000;
const byte SMS_MAX_RETRIES = 3;

// Fixed-size response buffer (saves Arduino RAM)
char response[120];
byte responseLength = 0;

void clearResponse() {
    responseLength = 0;
    response[0] = '\0';
}

bool hasResponse(const char* target) {
    return strstr(response, target) != NULL;
}

bool hasError() {
    return hasResponse("+CMS ERROR") ||
           hasResponse("+CME ERROR") ||
           hasResponse("\r\nERROR\r\n");
}

// LCD: only two screens (normal / shaking)
void showStatus(bool shakingDetected) {
    lcd.clear();

    if (!shakingDetected) {
        lcd.setCursor(0, 0);
        lcd.print(F("System Normal"));
        lcd.setCursor(0, 1);
        lcd.print(F("Monitoring..."));
    }
    else {
        lcd.setCursor(0, 0);
        lcd.print(F("Shaking detected"));
        lcd.setCursor(0, 1);
        lcd.print(F("Stay Alert!"));
    }
}

// Buzzer: one steady tone while shaking
void updateBuzzer() {
    if (alertActive) {
        tone(BUZZER_PIN, 1000);
    } else {
        noTone(BUZZER_PIN);
    }
}

void changeSMSState(SMSState next, unsigned long now) {
    smsState = next;
    smsStepTime = now;
    clearResponse();
}

void finishSMS(bool success, unsigned long now) {
    if (success) {
        smsSuccessCount++;

        Serial.println(F("\nSMS submitted successfully!"));

        Serial.print(F("Detection to SMS submission: "));
        Serial.print(now - detectionTime);
        Serial.println(F(" ms"));

        Serial.print(F("SMS processing time: "));
        Serial.print(now - smsStartTime);
        Serial.println(F(" ms"));

        smsPending = false;
        smsRetryCount = 0;
        smsRetryAfter = 0;
    }
    else {
        smsFailureCount++;
        smsRetryCount++;

        Serial.println(F("\nSMS attempt failed or timed out."));

        if (smsRetryCount >= SMS_MAX_RETRIES) {
            Serial.println(F("Max retries reached. SMS dropped."));
            smsPending = false;
            smsRetryCount = 0;
            smsRetryAfter = 0;
        } else {
            Serial.println(F("Will retry."));
            smsRetryAfter = now + SMS_RETRY_DELAY;
        }
    }

    changeSMSState(SMS_IDLE, now);
}

// Non-blocking SMS sender (does not stop sensor reading)
void updateSMS(unsigned long now) {
    while (sim800.available()) {
        char c = sim800.read();
        Serial.write(c);

        if (responseLength < sizeof(response) - 1) {
            response[responseLength++] = c;
            response[responseLength] = '\0';
        }
    }

    if (smsState == SMS_IDLE) {
        if (!smsPending) return;

        // Wait before retrying a failed SMS
        if (smsRetryAfter != 0 &&
            (long)(now - smsRetryAfter) < 0) {
            return;
        }

        smsStartTime = now;

        // Clear stale modem responses
        while (sim800.available()) {
            sim800.read();
        }

        clearResponse();

        sim800.println(F("AT+CMGF=1"));
        changeSMSState(SMS_WAIT_TEXT_MODE, now);

        Serial.println(F("\nPreparing SMS..."));
        return;
    }

    if (hasError()) {
        Serial.println(F("\nModem returned an error."));
        finishSMS(false, now);
        return;
    }

    if (smsState == SMS_WAIT_TEXT_MODE) {
        if (hasResponse("\r\nOK\r\n")) {
            sim800.print(F("AT+CMGS=\""));
            sim800.print(PHONE_NUMBER);
            sim800.println(F("\""));

            changeSMSState(SMS_WAIT_PROMPT, now);
        }
        else if (now - smsStepTime > 5000) {
            finishSMS(false, now);
        }
    }
    else if (smsState == SMS_WAIT_PROMPT) {
        if (hasResponse(">")) {
            clearResponse();

            sim800.print(F(
                "WARNING: Ground shaking detected. "
                "Stay alert and move to a safe location."
            ));

            sim800.write(26); // Ctrl+Z

            changeSMSState(SMS_WAIT_RESULT, now);
            Serial.println(F("\nSending SMS..."));
        }
        else if (now - smsStepTime > 10000) {
            finishSMS(false, now);
        }
    }
    else if (smsState == SMS_WAIT_RESULT) {
        if (hasResponse("+CMGS:") &&
            hasResponse("\r\nOK\r\n")) {
            finishSMS(true, now);
        }
        else if (now - smsStepTime > 60000) {
            finishSMS(false, now);
        }
    }
}

// One SMS per shaking event
void updateShakingEvent(bool detected, unsigned long now) {
    if (detected) {
        if (!shakeEventActive) {
            shakeEventActive = true;
            eventCount++;
            detectionTime = now;

            Serial.println(F("\nNEW SHAKING EVENT"));

            if (!smsPending) {
                smsPending = true;
                smsRetryCount = 0;
                smsRetryAfter = 0;
            }
        }

        lastShakeTime = now;
    }
    else if (shakeEventActive &&
             now - lastShakeTime > SHAKE_GAP) {

        shakeEventActive = false;
        Serial.println(F("Shaking event ended."));
    }
}

void printResearchData(unsigned long now) {
    if (now - lastPrintTime < PRINT_INTERVAL) return;
    lastPrintTime = now;

    Serial.print(F("Acceleration change: "));
    Serial.print(shaking, 3);
    Serial.print(F(" m/s^2"));

    Serial.print(F(" | Peak: "));
    Serial.print(peakShaking, 3);

    Serial.print(F(" | Alert: "));
    Serial.print(alertActive ? 1 : 0);

    Serial.print(F(" | Events: "));
    Serial.print(eventCount);

    Serial.print(F(" | SMS accepted: "));
    Serial.print(smsSuccessCount);

    Serial.print(F(" | SMS failed: "));
    Serial.println(smsFailureCount);
}

void setup() {
    Serial.begin(9600);
    sim800.begin(9600);

    pinMode(BUZZER_PIN, OUTPUT);
    noTone(BUZZER_PIN);

    Wire.begin();

    lcd.init();
    lcd.backlight();

    lcd.setCursor(0, 0);
    lcd.print(F("Shaking Monitor"));
    lcd.setCursor(0, 1);
    lcd.print(F("Starting..."));

    if (!mpu.begin()) {
        lcd.clear();
        lcd.print(F("MPU6050 ERROR"));
        Serial.println(F("MPU6050 not found!"));

        while (true) {
            delay(100);
        }
    }

    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

    clearResponse();

    Serial.println(F("Ground Shaking Monitor Ready"));

    showStatus(false);
}

void loop() {
    unsigned long now = millis();

    // These keep running while an SMS is being sent
    updateSMS(now);

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

    float dx = x - prevX;
    float dy = y - prevY;
    float dz = z - prevZ;

    rawShaking = sqrt(dx * dx + dy * dy + dz * dz);

    shaking = FILTER_ALPHA * rawShaking +
              (1.0 - FILTER_ALPHA) * shaking;

    prevX = x;
    prevY = y;
    prevZ = z;

    if (shaking > peakShaking) {
        peakShaking = shaking;
    }

    bool detected = (shaking >= SHAKE_THRESHOLD);

    // Timestamp immediately after classifying shaking
    now = millis();

    updateShakingEvent(detected, now);

    bool oldAlert = alertActive;

    if (detected) {
        alertActive = true;
    }
    else if (alertActive && now - lastShakeTime >= HOLD_TIME) {
        alertActive = false;
    }

    if (alertActive != oldAlert) {
        // Activate buzzer before updating LCD
        unsigned long buzzerCommandTime = millis();
        updateBuzzer();

        if (alertActive) {
            Serial.print(F("Detection to buzzer command: "));
            Serial.print(buzzerCommandTime - now);
            Serial.println(F(" ms"));
        }

        showStatus(alertActive);
    }

    printResearchData(now);
}