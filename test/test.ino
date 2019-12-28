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
    Serial1.begin(10400);
    Serial2.begin(10400);
//    com.begin(10400);
}

unsigned long time;

const byte dataLpg[5] = {0xC1, 0x33, 0xF1, 0x81, 0x66};

void initPulse() {
    Serial2.end();
    pinMode(16, OUTPUT);
    digitalWrite(16, LOW);
    delay(25);
    digitalWrite(16, HIGH);
    delay(25);
    Serial2.begin(10400);
}

void initData(byte *data, uint8_t size) {
    for (uint8_t i = 0; i < size; ++i) {
        Serial2.write(data[i]);
        Serial.print(" ");
        Serial.print(data[i], HEX);
        delay(5);
    }
}


void loop() {


    if (time + 2500 < millis()) {
        Serial.println(" ");
        Serial.print(" |> ");
        initPulse();
        initData(dataLpg, 5);
        time = millis();
        Serial.println(" ");
        Serial.print(" |< ");
    }
    while (Serial2.available()) {
        Serial.print(" ");
        Serial.print(Serial2.read(), HEX);
    }
}