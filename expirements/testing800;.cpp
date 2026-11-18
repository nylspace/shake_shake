
#include <Arduino.h>
#include <SoftwareSerial.h>

SoftwareSerial sim800(10, 11);

void setup() {
    Serial.begin(9600);
    sim800.begin(9600);

    delay(5000);

    Serial.println("Starting SMS test...");

    sim800.println("AT+CMGF=1");
    delay(1000);

    sim800.println("AT+CMGS=\"+639760780477\"");
    delay(2000);

    sim800.print("Earthquake Alert! Shaking detected.");

    sim800.write(26);  // Ctrl+Z sends SMS

    Serial.println("SMS submitted. Waiting for response...");
}

void loop() {
    while (sim800.available()) {
        Serial.write(sim800.read());
    }
}
