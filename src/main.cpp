
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

Adafruit_MPU6050 mpu;
LiquidCrystal_I2C lcd(0x27, 16, 2);

// SIM800L: Arduino RX, TX
SoftwareSerial sim800(10, 11);
const char PHONE_NUMBER[] = "+639760780477";

// Buzzers
const int passiveBuzzer = 8;
const int activeBuzzer = 9;

// Shaking thresholds (m/s^2)
const float LIGHT_THRESHOLD = 1.5;
const float HEAVY_THRESHOLD = 4.0;

// Filtering and timing
const float FILTER_ALPHA = 0.20;
const unsigned long HOLD_TIME = 1500;
const unsigned long SAMPLE_INTERVAL = 20;
const unsigned long SMS_COOLDOWN = 60000;

// Acceleration values
float prevX = 0;
float prevY = 0;
float prevZ = 0;
float shaking = 0;

bool firstReading = true;
int alertLevel = 0;

unsigned long lastShakeTime = 0;
unsigned long lastSampleTime = 0;

// SMS tracking
unsigned long lastLightSMS = 0;
unsigned long lastHeavySMS = 0;
bool lightSMSSent = false;
bool heavySMSSent = false;

// Nonblocking SMS state
enum SMSState {
    SMS_IDLE,
    SMS_WAIT_PROMPT,
    SMS_WAIT_RESULT
};

SMSState smsState = SMS_IDLE;
unsigned long smsStepTime = 0;
int pendingSMSLevel = 0;

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

void updateBuzzers(unsigned long now) {
    if (alertLevel == 0) {
        noTone(passiveBuzzer);
        digitalWrite(activeBuzzer, LOW);
    }
    else if (alertLevel == 1) {
        tone(passiveBuzzer, 1000);
        digitalWrite(activeBuzzer, LOW);
    }
    else {
        // Passive buzzer uses tone().
        tone(passiveBuzzer, 1500);

        // Active buzzer receives ON/OFF voltage.
        bool beepOn = (now / 200) % 2 == 0;
        digitalWrite(activeBuzzer, beepOn ? HIGH : LOW);
    }
}

void requestSMS(int level, unsigned long now) {
    if (level == 0) return;

    if (level == 1) {
        if (!lightSMSSent ||
            now - lastLightSMS >= SMS_COOLDOWN) {
            if (pendingSMSLevel < 1)
                pendingSMSLevel = 1;
        }
    }

    if (level == 2) {
        if (!heavySMSSent ||
            now - lastHeavySMS >= SMS_COOLDOWN) {
            pendingSMSLevel = 2;
        }
    }
}


void updateSMS(unsigned long now) {
    static bool cmgsSent = false;
    static int sendingLevel = 0;
    static String response = "";

    // Read SIM800L responses
    while (sim800.available()) {
        char c = sim800.read();
        Serial.write(c);
        response += c;

        if (response.length() > 120) {
            response.remove(0, 60);
        }
    }

    // Start sending SMS
    if (smsState == SMS_IDLE && pendingSMSLevel > 0) {
        sendingLevel = pendingSMSLevel;
        pendingSMSLevel = 0;

        response = "";
        cmgsSent = false;

        sim800.println("AT+CMGF=1");

        smsState = SMS_WAIT_PROMPT;
        smsStepTime = now;

        Serial.println("Preparing SMS...");
    }

    // Send recipient number
    if (smsState == SMS_WAIT_PROMPT &&
        !cmgsSent &&
        now - smsStepTime >= 500) {

        response = "";

        sim800.print("AT+CMGS=\"");
        sim800.print(PHONE_NUMBER);
        sim800.println("\"");

        cmgsSent = true;
        smsStepTime = now;
    }

    // Wait for > prompt
    if (smsState == SMS_WAIT_PROMPT && cmgsSent) {
        if (response.indexOf(">") >= 0) {
            response = "";

            if (sendingLevel == 2) {
                sim800.print(
                    "EMERGENCY: Heavy shaking detected! "
                    "Evacuate now and stay safe."
                );
            } else {
                sim800.print(
                    "WARNING: Shaking detected by "
                    "the earthquake monitoring system."
                );
            }

            sim800.write(26); // Ctrl+Z

            smsState = SMS_WAIT_RESULT;
            smsStepTime = now;

            Serial.println("Sending SMS...");
        }
        else if (now - smsStepTime > 10000) {
            Serial.println("ERROR: No SMS prompt");
            smsState = SMS_IDLE;
        }
    }

    // Wait for confirmation
    if (smsState == SMS_WAIT_RESULT) {
        if (response.indexOf("+CMGS:") >= 0) {
            Serial.println("SMS accepted by network!");

            if (sendingLevel == 1) {
                lastLightSMS = now;
                lightSMSSent = true;
            } else {
                lastHeavySMS = now;
                heavySMSSent = true;
            }

            response = "";
            smsState = SMS_IDLE;
        }
        else if (response.indexOf("ERROR") >= 0) {
            Serial.println("SMS failed!");
            response = "";
            smsState = SMS_IDLE;
        }
        else if (now - smsStepTime > 30000) {
            Serial.println("SMS timeout!");
            smsState = SMS_IDLE;
        }
    }
}


void setup() {
    Serial.begin(9600);
    sim800.begin(9600);

    pinMode(passiveBuzzer, OUTPUT);
    pinMode(activeBuzzer, OUTPUT);

    noTone(passiveBuzzer);
    digitalWrite(activeBuzzer, LOW);

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

    Serial.println("Initializing SIM800L...");

    sim800.println("AT");
    delay(1000);

    while (sim800.available()) {
        Serial.write(sim800.read());
    }

    delay(1000);
    showStatus(0);
}

void loop() {
    unsigned long now = millis();

    updateSMS(now);
    updateBuzzers(now);

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

    float rawShaking = sqrt(
        dx * dx + dy * dy + dz * dz
    );

    shaking = FILTER_ALPHA * rawShaking +
              (1.0 - FILTER_ALPHA) * shaking;

    prevX = x;
    prevY = y;
    prevZ = z;

    int detectedLevel = 0;

    if (shaking >= HEAVY_THRESHOLD) {
        detectedLevel = 2;
    }
    else if (shaking >= LIGHT_THRESHOLD) {
        detectedLevel = 1;
    }

    if (detectedLevel > 0) {
        lastShakeTime = now;
        requestSMS(detectedLevel, now);
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

    // Display sensor readings every 1 second
    // static unsigned long lastPrintTime = 0;

    // if (now - lastPrintTime >= 1000) {
    //     lastPrintTime = now;

    //     Serial.print("Shaking: ");
    //     Serial.print(shaking);
    //     Serial.print(" | Alert: ");
    //     Serial.println(alertLevel);
    // }
}
