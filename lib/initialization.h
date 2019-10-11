//
// Created by Angel Zaprianov on 2019-10-10.
//

#ifndef KLINECAN_INITIALIZATION_H
#define KLINECAN_INITIALIZATION_H

#include "header.h"

void lpgInitSend(byte check, byte send, uint16_t timeout) {
    uint8_t _sync;
    if (!getSoftSerial(_sync, timeout)) {
        if (_sync == check) {
            ecuLine.write(send);
        }
    }
}

boolean fastInitFrame() {
    lpgInitSend(0xC1, 0x83, 1500);
    lpgInitSend(0x33, 0xF1, 15);
    lpgInitSend(0xF1, 0x11, 15);
    lpgInitSend(0x81, 0xC1, 15);
    lpgInitSend(0x66, 0xEF, 15);
    ecuLine.write(0xC4); // checksum
}

void lpgInit() {
    ecuLine.flushInput();
    ecuLine.begin(10400);
    Serial.println(F("Waiting fast  init "));

    /*
      0xC1 0x33 0xF1 0x81 0x66 is transmitted by the UART to the car
     */
    uint8_t income = ecuLine.read();
    Serial.println(income);
    if (income == 0xC1) {
        Serial.println("OK");
    }
    /*
     The car responds with 0x83 0xF1 0x11 0xC1 0x8F 0xEF 0xC4 (sum of all received bytes except the last should be 0xC4).
     */
    uint8_t data[8] = {0x83, 0xF1, 0x11, 0xC1, 0x8F, 0xEF, 0xC4};
    sendSoftSerial(data, 7, 25);

    uint8_t sync;
    uint16_t timeout = 5000;
    if (!getSoftSerial(sync, timeout)) {
        snprintf_P(buffer, BUFLEN, PSTR("SYNC Byte: %2X"), sync);
        Serial.println(buffer);
    }
}

void ecuInit() {
    digitalWrite(txPin, LOW); // start bit
    digitalWrite(LED_BUILTIN, LOW);
    delay(25);
    digitalWrite(txPin, HIGH); //11
    digitalWrite(LED_BUILTIN, HIGH);
    ecuLine.flushInput();
    ecuLine.begin(10400);
    delay(25);// or 20

//    uint8_t data[5] = {0x81, 0x11, 0xF1, 0x81, 0x04};
    uint8_t data[5] = {129, 17, 241, 129, 4};

    sendSoftSerial(data, 5);
    delay(25);

    Serial.println("Waiting response");

    bool bInitError = false;  // If there is an error, set bInitError to true
    String strError = "";     // and store the error description in strError.
    bool iso9141 = false;     // Will either be ISO 9141 or ISO 14230
    // Wait for SYNC byte (0x55)
    uint8_t syncStart, syncEnd;
    uint16_t timeout = 1000;
    uint32_t start = millis();
    while (!ecuLine.available()) {
        //Wait for a byte to arrive
        if ((millis() - start) > timeout) {

        }
    }
    syncStart = ecuLine.read();
    if (syncStart == 131) {
        snprintf_P(buffer, BUFLEN, PSTR("SYNC Byte: %2X"), syncStart);
        Serial.println(buffer);
        uint8_t tries[3] = {241, 193, 143}; // 0xF1 0xC1 0xC4
        uint8_t sync;
        sendSoftSerial(tries, 3);

    }
    syncEnd = ecuLine.read();
    if (syncEnd == 196) {
        snprintf_P(buffer, BUFLEN, PSTR("SYNC ended: %2X"), syncEnd);
        Serial.println(buffer);
    }


    /*
     * To lpg ecu
     * Session		and	teardown
     *      0x81	Start		Service	Request
     *      0x82	Stop		Service	Request
     *
     *      0x01 : show current data
     *      0x02 : show freeze frame data
     *      0x03 : show stored diagnostic trouble code.
     *
     *      0x04 : engine load
     *      0x05 : engine coolant temparature
     *      0x0C : engine rpm
     *      0x0D : vehicle spped
     *      0x10 : MAF air flow rate
     *
     *
     *      LPG initialise
     *      From SDS: 81 12 F1 81 05
     *      From ECU: 80 F1 12 03 C1 EA 8F C0
     * */
}


#endif //KLINECAN_INITIALIZATION_H
