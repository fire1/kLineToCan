#include <Arduino.h>
#include <AltSoftSerial.h>


#define txPin 9
#define rxPin 8

#ifndef  AltSoftSerial_h

#include "../libraries/AltSoftSerial/AltSoftSerial.h"

#endif


AltSoftSerial kLine(txPin, rxPin); // Uses Tx = 9, Rx = 8

const uint8_t pinEcu = A0;
const uint8_t pinLpg = A1;

boolean whileInit() {
    unsigned long in;
    in = pulseIn(rxPin, LOW, 2000000);
    return in > 24000 && in < 26000;
}

/**
 * A function that will send (Tx) an array of bytes to the Software Serial port
 * @param bytes
 * @param size
 * @param pause
 */
void writeInit(uint8_t *bytes, unsigned size, uint8_t pause = 5) {
    for (uint8_t i = 0; i < size; i++) {
        // Send the next byte
        kLine.write(bytes[i]);
        // TX and RX share the same line so we recieve back the byte we just sent
        delay(pause); // P4 (Inter-byte spacing)
        delayMicroseconds(800);
    }
    kLine.flushInput();
}

void setup() {
    Serial.begin(115200);
    pinMode(rxPin, INPUT);
    pinMode(txPin, OUTPUT);
    pinMode(txPin, HIGH);
    Serial.println(F("Booting ..."));
    pinMode(pinEcu, OUTPUT);
    pinMode(pinLpg, OUTPUT);
    digitalWrite(pinEcu, HIGH);
    digitalWrite(pinLpg, HIGH);
    kLine.begin(10400);
    delay(5000);

}


// LPG init loop is ~4sec

// Send to ECU opel         0x81 0x11 0xF1 0x81 0x04
// Send to ECU k-line       0xC1 0x33 0xF1 0x81 0x66
// Response from car    0x83 0xF1 0x11 0xC1 0x8F 0xEF 0xC4


byte ecuData[5] = {0x81, 0x11, 0xF1, 0x81, 0x04};
byte lpgData[7] = {0x83, 0xF1, 0x11, 0xC1, 0x8F, 0xEF, 0xC4};
byte ecu2Data[6] = {0x82, 0x11, 0xF1, 0x21, 0x01, 0xA6};
byte ecu2Data_[6] = {0x82, 0x11, 0xF1, 0x01, 0x00, 0x85};

void loop_() {
    digitalWrite(pinEcu, LOW);
    writeInit(lpgData, 7);
    delay(1000);
}

void loop() {
    digitalWrite(LED_BUILTIN, LOW);
    digitalWrite(pinEcu, HIGH);
    digitalWrite(pinLpg, HIGH);
    while (!whileInit());
    digitalWrite(LED_BUILTIN, HIGH);
    delay(25);
//    delayMicroseconds(100);
    digitalWrite(pinLpg, LOW);
    writeInit(ecuData, 5);
    digitalWrite(pinLpg, HIGH);
    digitalWrite(LED_BUILTIN, LOW);

    delay(56);
    digitalWrite(LED_BUILTIN, HIGH);

    digitalWrite(pinLpg, HIGH);
    digitalWrite(pinEcu, LOW);
    writeInit(lpgData, 7);

    delay(1);
    digitalWrite(pinLpg, LOW);
    digitalWrite(pinEcu, HIGH);
    writeInit(ecu2Data, 6);

    delay(1);
    digitalWrite(pinEcu, HIGH);
    digitalWrite(pinLpg, HIGH);
}