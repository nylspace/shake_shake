
#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

//scl to a5 and sda to a4 on arduino uno
//vcc to 5v and gnd to gnd on arduino uno

LiquidCrystal_I2C lcd(0x27, 16, 2);

const int buttonPin = 2;
const int buzzerPin = 8;

void setup() {
    pinMode(buttonPin, INPUT_PULLUP);
    pinMode(buzzerPin, OUTPUT);
    noTone(buzzerPin);
    Serial.begin(9600);

    lcd.init();
    lcd.backlight();

    lcd.setCursor(0, 0);
    lcd.print("Button Status:");
}

void loop() {
    bool pressed = digitalRead(buttonPin) == LOW;

    lcd.setCursor(0, 1);

    if (pressed) {
        lcd.print("PRESSED         ");
        tone(buzzerPin, 1000);
    } else {
        lcd.print("RELEASED        ");
        noTone(buzzerPin);
    }

    delay(50);
}
