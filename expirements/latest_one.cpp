
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

// Acceleration-change thresholds (m/s^2)
const float LIGHT_THRESHOLD = 1.5;
const float HEAVY_THRESHOLD = 4.0;
const float FILTER_ALPHA = 0.20;

// Timing
const unsigned long SAMPLE_INTERVAL = 20;
const unsigned long HOLD_TIME = 1500;
const unsigned long CONTINUOUS_TIME = 5000;
const unsigned long PRINT_INTERVAL = 1000;
const unsigned long SHAKE_GAP = 500;

// Sensor data
float prevX = 0, prevY = 0, prevZ = 0;
float shaking = 0;
float rawShaking = 0;

bool firstReading = true;
int alertLevel = 0;
int previousDetectedLevel = 0;

unsigned long lastSampleTime = 0;
unsigned long lastShakeTime = 0;
unsigned long shakeStartTime = 0;
unsigned long lastPrintTime = 0;

bool shakeEventActive = false;
bool continuationQueued = false;
bool heavySMSQueued = false;

// Research statistics
unsigned long eventCount = 0;
unsigned long smsSuccessCount = 0;
unsigned long smsFailureCount = 0;
float peakShaking = 0;

float trialPeak = 0;
float trialSum = 0;
unsigned long trialSamples = 0;
unsigned long trialStartTime = 0;
unsigned long trialNumber = 0;

// SMS queue
enum MessageType {
    MSG_LIGHT,
    MSG_CONTINUED,
    MSG_HEAVY
};

const byte QUEUE_SIZE = 6;
MessageType smsQueue[QUEUE_SIZE];
byte queueCount = 0;

enum SMSState {
    SMS_IDLE,
    SMS_WAIT_TEXT_MODE,
    SMS_WAIT_PROMPT,
    SMS_WAIT_RESULT
};

SMSState smsState = SMS_IDLE;

MessageType currentMessage = MSG_LIGHT;

unsigned long smsStepTime = 0;
unsigned long smsStartTime = 0;
unsigned long currentDetectionTime = 0;

unsigned long smsRetryAfter = 0;
unsigned long smsRetryCount = 0;

const unsigned long SMS_RETRY_DELAY = 5000;

// Keep detection timestamp for each queued SMS
unsigned long queueTimes[QUEUE_SIZE];

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

bool queueSMS(MessageType type, unsigned long detectedAt) {
    if (queueCount >= QUEUE_SIZE) {
        Serial.println(F("SMS queue full!"));
        return false;
    }

    smsQueue[queueCount] = type;
    queueTimes[queueCount] = detectedAt;
    queueCount++;

    Serial.print(F("SMS queued. Waiting: "));
    Serial.println(queueCount);

    return true;
}

void showStatus(int level) {
    lcd.clear();

    if (level == 0) {
        lcd.setCursor(0, 0);
        lcd.print(F("System Normal"));
        lcd.setCursor(0, 1);
        lcd.print(F("Monitoring..."));
    }
    else if (level == 1) {
        lcd.setCursor(0, 0);
        lcd.print(F("Shaking detected"));
        lcd.setCursor(0, 1);
        lcd.print(F("Stay Alert!"));
    }
    else {
        lcd.setCursor(0, 0);
        lcd.print(F("Heavy Shaking"));
        lcd.setCursor(0, 1);
        lcd.print(F("Evacuate Now!"));
    }
}

// Only one passive buzzer
void updateBuzzer(unsigned long now) {
    if (alertLevel == 0) {
        noTone(BUZZER_PIN);
    }
    else if (alertLevel == 1) {
        tone(BUZZER_PIN, 1000);
    }
    else {
        // Alternating emergency notes
        if ((now / 180) % 2 == 0) {
            tone(BUZZER_PIN, 1800);
        } else {
            tone(BUZZER_PIN, 2600);
        }
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
        Serial.print(now - currentDetectionTime);
        Serial.println(F(" ms"));

        Serial.print(F("SMS processing time: "));
        Serial.print(now - smsStartTime);
        Serial.println(F(" ms"));

        // Remove the confirmed message from the queue
        for (byte i = 1; i < queueCount; i++) {
            smsQueue[i - 1] = smsQueue[i];
            queueTimes[i - 1] = queueTimes[i];
        }

        if (queueCount > 0) {
            queueCount--;
        }

        smsRetryCount = 0;
        smsRetryAfter = 0;
    }
    else {
        smsFailureCount++;
        smsRetryCount++;

        Serial.println(F("\nSMS attempt failed or timed out."));
        Serial.println(F("Message retained for retry."));

        smsRetryAfter = now + SMS_RETRY_DELAY;
    }

    changeSMSState(SMS_IDLE, now);
}

// Processes only one SMS at a time
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
        if (queueCount == 0) return;

        // Wait before retrying a failed SMS
        if (smsRetryAfter != 0 &&
            (long)(now - smsRetryAfter) < 0) {
            return;
        }

        // Read the first message without removing it
        currentMessage = smsQueue[0];
        currentDetectionTime = queueTimes[0];

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

            if (currentMessage == MSG_LIGHT) {
                sim800.print(F(
                    "WARNING: Ground shaking detected. "
                    "Stay alert."
                ));
            }
            else if (currentMessage == MSG_CONTINUED) {
                sim800.print(F(
                    "WARNING: Ground shaking has continued "
                    "for at least 5 seconds."
                ));
            }
            else {
                sim800.print(F(
                    "EMERGENCY: Heavy ground shaking "
                    "detected! Evacuate to a safe location."
                ));
            }

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

void updateShakingEvent(int detectedLevel, unsigned long now) {
    if (detectedLevel > 0) {
        if (!shakeEventActive) {
            shakeEventActive = true;
            continuationQueued = false;
            heavySMSQueued = false;
            shakeStartTime = now;
            eventCount++;

            Serial.println(F("\nNEW SHAKING EVENT"));

            // First warning immediately
            if (detectedLevel == 2) {
                queueSMS(MSG_HEAVY, now);
                heavySMSQueued = true;
            } else {
                queueSMS(MSG_LIGHT, now);
            }
        }
        else {
            // Worsening from light to heavy
            if (!heavySMSQueued && detectedLevel == 2) {
                heavySMSQueued = queueSMS(MSG_HEAVY, now);

                if (heavySMSQueued) {
                    Serial.println(F("Shaking worsened: emergency SMS queued."));
                }
            }
        }

        lastShakeTime = now;

        // One follow-up after 5 seconds
        if (!continuationQueued &&
            now - shakeStartTime >= CONTINUOUS_TIME) {

            continuationQueued = queueSMS(MSG_CONTINUED, now);

            if (continuationQueued) {
                Serial.println(F("Continuous shaking: follow-up SMS queued."));
            }
        }

        previousDetectedLevel = detectedLevel;
    }
    else if (shakeEventActive &&
             now - lastShakeTime > SHAKE_GAP) {

        shakeEventActive = false;
        heavySMSQueued = false;
        previousDetectedLevel = 0;
        continuationQueued = false;

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
    Serial.print(alertLevel);

    Serial.print(F(" | Events: "));
    Serial.print(eventCount);

    Serial.print(F(" | SMS queued: "));
    Serial.print(queueCount);

    Serial.print(F(" | SMS accepted: "));
    Serial.print(smsSuccessCount);

    Serial.print(F(" | SMS failed: "));
    Serial.println(smsFailureCount);
}

void recordTrial(int detectedLevel, unsigned long now) {
    static bool recording = false;
    static unsigned long lastDetectedTime = 0;

    if (detectedLevel > 0) {
        if (!recording) {
            recording = true;
            trialNumber++;
            trialStartTime = now;

            trialPeak = 0;
            trialSum = 0;
            trialSamples = 0;

            Serial.println();
            Serial.print(F("TRIAL #"));
            Serial.print(trialNumber);
            Serial.println(F(" STARTED"));
        }

        lastDetectedTime = now;
    }

    if (recording) {
        trialSamples++;
        trialSum += shaking;

        if (shaking > trialPeak) {
            trialPeak = shaking;
        }

        // End trial after 1.5 seconds without detected shaking
        if (detectedLevel == 0 &&
            now - lastDetectedTime >= 1500) {

            recording = false;

            Serial.println(F("\n===== TRIAL RESULT ====="));

            Serial.print(F("Trial: "));
            Serial.println(trialNumber);

            Serial.print(F("Peak acceleration change: "));
            Serial.print(trialPeak, 3);
            Serial.println(F(" m/s^2"));

            Serial.print(F("Average acceleration change: "));
            Serial.print(trialSum / trialSamples, 3);
            Serial.println(F(" m/s^2"));

            Serial.print(F("Duration: "));
            Serial.print(now - trialStartTime);
            Serial.println(F(" ms"));

            Serial.print(F("Classification: "));

            if (trialPeak >= HEAVY_THRESHOLD) {
                Serial.println(F("HEAVY"));
            }
            else {
                Serial.println(F("LIGHT"));
            }

            Serial.println(F("========================\n"));
        }
    }
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
    Serial.println(F("SIM800L SMS queue enabled"));

    showStatus(0);
}

void loop() {
    unsigned long now = millis();

    // These continue operating while SMS is processing
    updateSMS(now);
    updateBuzzer(now);

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

    rawShaking = sqrt(
        dx * dx + dy * dy + dz * dz
    );

    shaking = FILTER_ALPHA * rawShaking +
              (1.0 - FILTER_ALPHA) * shaking;

    prevX = x;
    prevY = y;
    prevZ = z;

    if (shaking > peakShaking) {
        peakShaking = shaking;
    }

    int detectedLevel = 0;

    if (shaking >= HEAVY_THRESHOLD) {
        detectedLevel = 2;
    }
    else if (shaking >= LIGHT_THRESHOLD) {
        detectedLevel = 1;
    }

    // Timestamp immediately after classifying shaking
    now = millis();

    updateShakingEvent(detectedLevel, now);
    recordTrial(detectedLevel, now);

    int oldAlertLevel = alertLevel;

    if (detectedLevel > alertLevel) {
        alertLevel = detectedLevel;
    }
    else if (detectedLevel < alertLevel &&
             now - lastShakeTime >= HOLD_TIME) {
        alertLevel = detectedLevel;
    }

    if (alertLevel != oldAlertLevel) {
        unsigned long buzzerCommandTime = millis();

        // Activate buzzer before updating LCD
        updateBuzzer(buzzerCommandTime);

        if (alertLevel > oldAlertLevel) {
            Serial.print(F("Detection to buzzer command: "));
            Serial.print(buzzerCommandTime - now);
            Serial.println(F(" ms"));
        }

        showStatus(alertLevel);
    }

    printResearchData(now);
}
