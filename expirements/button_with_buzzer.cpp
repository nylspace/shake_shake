#include <Arduino.h>

const int buttonPin = 2;
const int buzzerPin = 8;

void setup() {
    pinMode(buttonPin, INPUT_PULLUP);
    pinMode(buzzerPin, OUTPUT);
    noTone(buzzerPin);
    Serial.begin(9600);
}

void loop() {
    int buttonState = digitalRead(buttonPin);

    if (buttonState == LOW) {
        Serial.println("PRESSED");
        tone(buzzerPin, 1000);
    } else {
        Serial.println("RELEASED");
        noTone(buzzerPin);
    }

    delay(100);
}
