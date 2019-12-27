//
// Created by Angel Zaprianov on 2019-10-10.
//

#ifndef KLINECAN_INITIALIZATION_H
#define KLINECAN_INITIALIZATION_H

#include "header.h"

void lpgInitSend(byte check, byte send, uint16_t timeout) {
    uint8_t _sync;
    if (!getLpgSerial(_sync, timeout)) {
        if (_sync == check) {
            Serial.print(" / send ");
            Serial.print(send, HEX);
            kLine.write(send);
            getLpgSerial(send, 1);
        } else {
            kLine.write(send);
            Serial.println(" Error ");
            Serial.print(check, HEX);

        }
    }
}

/**
 *
 * @param beginByte
 * @param responseByte
 * @return
 */
boolean lpgFindBegin(byte beginByte = 0xC1, byte responseByte = 0x83, uint16_t timeout = 3000) {
    uint8_t first;
    if (!getLpgSerial(first, timeout)) {
        if (first == beginByte) {
            Serial.print(" starting ");
            Serial.print(first, HEX);
            Serial.print(" / send ");
            Serial.print(responseByte, HEX);
            kLine.write(responseByte); // first response
            getLpgSerial(first, 1);
            return true;
        }
    }
    return false;
}

boolean pulseStart() {
    unsigned long in = pulseIn(rxPin, LOW, 3000);
    Serial.print(" cap ");
    Serial.print(in);
    return in == 25;
}


void mixStart() {

    // Send to opel         0x81 0x11 0xF1 0x81 0x04
    // Send to k-line       0xC1 0x33 0xF1 0x81 0x66
    // Response from car    0x83 0xF1 0x11 0xC1 0x8F 0xEF 0xC4

    digitalWrite(pinEcu, HIGH);
    digitalWrite(pinLpg, HIGH);
    while (!pulseStart());
    delay(23);
    digitalWrite(pinEcu, HIGH);
    digitalWrite(pinLpg, LOW);
    uint8_t data[5] = {0x81, 0x11, 0xF1, 0x81, 0x04};
    sendSoftSerial(data, 5);
    digitalWrite(pinLpg, HIGH);


}

void lpgInit() {

    //
    // Send to car          0xC1 0x33 0xF1 0x81 0x66
    // Response from car    0x83 0xF1 0x11 0xC1 0x8F 0xEF 0xC4
    while (!lpgFindBegin(0xC1, 0x83, 1500));


    uint8_t temp;
    while (!lpgFindBegin(0xC1, 0x83, 1500));
    lpgInitSend(0x33, 0xF1, 15);
    lpgInitSend(0xF1, 0x11, 15);
    lpgInitSend(0x81, 0xC1, 15);
    lpgInitSend(0x66, 0xEF, 15);

    // May be ?
    kLine.write(0xC4); // checksum
    getLpgSerial(temp, 1);

//    uint8_t data2[6] = {0x82, 0x11, 0xF1, 0x21, 0x01, 0xA6}; // secondary data
//    sendSoftSerial(data2, 6);
    // response 0x83 0xF1 0x11 0xC1 0x8F 0xEF 0xC4
}

boolean ecuInit() {
    digitalWrite(txPin, LOW); // start bit
    digitalWrite(LED_BUILTIN, LOW);
    delay(25);
    digitalWrite(txPin, HIGH); //11
    digitalWrite(LED_BUILTIN, HIGH);
//    kLine.flushInput();
    delay(25);// or 20


    uint8_t data[5] = {0x81, 0x11, 0xF1, 0x81, 0x04};
    sendSoftSerial(data, 5);
    // need response 0x83 0xF1 0x11 0xC1 0x8F 0xEF 0xC4
    byte syncByte[1];
    if (getSoftSerial(syncByte[1], 2000)) {

        digitalWrite(pinEcu, LOW);
        digitalWrite(pinLpg, HIGH);
        sendSoftSerial(syncByte, 1);
        snprintf_P(buffer, BUFLEN, PSTR("ECU Sync: %2X"), syncByte);
        Serial.println(buffer);
    }
    return true;
    Serial.println("Waiting ECU response");


    String strError = "";     // and store the error description in strError.
    // Wait for SYNC byte (0x55)
    uint8_t syncStart, syncEnd;


    getSoftSerial(syncStart, 2000);

    if (syncStart !=/*  == 0x83*/ 0x00) {
        snprintf_P(buffer, BUFLEN, PSTR("ECU Capture starting: %2X"), syncStart);
        Serial.println(buffer);

// old op-com
//        uint8_t tries[3] = {0xF1, 0xC1, 0xC4}; // respond 0x11 0xEF
        uint8_t data1[3] = {0xF1 /* 0x11 */, 0xC1 /* 0xEF */, 0x8F /* 0xC4 */}; //
        sendSoftSerial(data1, 3);
//        delay(10);
        return true;
//        return true;
    } else {
        snprintf_P(buffer, BUFLEN, PSTR("Wrong ECU: %2X"), syncStart);
        Serial.println(buffer);

    }


    getSoftSerial(syncEnd, 2000);
    if (syncEnd != /* == 239*//*196*/ 0x00) {
        snprintf_P(buffer, BUFLEN, PSTR(" ECU  ended: %2X"), syncEnd);
        Serial.println(buffer);
        return true;
    } else {
        Serial.println("Ended ");
        Serial.print(syncEnd);
        Serial.println();
    }

    return false;
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
