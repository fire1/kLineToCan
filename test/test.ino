#include <Arduino.h>
#include <AltSoftSerial.h>

#define txPin 9
#define rxPin 8
// Must use the same UART pins from AltSoftSerial
AltSoftSerial com(txPin, rxPin); // Uses Tx = 9, Rx = 8

#ifdef AltSoftSerial_h
#include "../../libraries/AltSoftSerial/AltSoftSerial.h"
#endif

void setup() {
    Serial.begin(115200);
    com.begin(10400);
}

unsigned long time;

void loop() {


    if (com.available()) {
        Serial.println(com.read());
        time = millis();
    }

    if (time + 3000 < millis()) {
        com.print("Testing ");
        time = millis();
    }
}