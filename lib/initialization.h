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
            Serial.print(" / send ");
            Serial.print(send, HEX);
            ecuLine.write(send);
            getSoftSerial(send, 1);
        } else {
            ecuLine.write(send);
            Serial.println(" Error ");
            Serial.print(check, HEX);

        }
    }
}

boolean lpgFindBegin() {
    uint8_t first;
    if (!getSoftSerial(first, 3000)) {
        if (first == 0xC1) {
            Serial.print(" starting ");
            Serial.print(first, HEX);
            Serial.print(" / send ");
            Serial.print(0x83, HEX);
            ecuLine.write(0x83); // first response
            getSoftSerial(first, 1);
            return true;
        }

        Serial.print(" reads: ");
        Serial.print(first, HEX);
    }
    return false;
}

boolean fastInitFrame() {

    uint8_t temp;
    while (!lpgFindBegin());
    lpgInitSend(0x33, 0xF1, 15);
    lpgInitSend(0xF1, 0x11, 15);
    lpgInitSend(0x81, 0xC1, 15);
    lpgInitSend(0x66, 0xEF, 15);

    ecuLine.write(0xC4); // checksum
    getSoftSerial(temp, 1);
    Serial.println();
    Serial.println(F(" Done!"));


}


void lpgInit() {
    Serial.println(F("LPG fast init ... "));

    fastInitFrame();
    Serial.println(F("Finish LPG ..."));
}

boolean ecuInit() {
    digitalWrite(txPin, LOW); // start bit
    digitalWrite(LED_BUILTIN, LOW);
    delay(25);
    digitalWrite(txPin, HIGH); //11
    digitalWrite(LED_BUILTIN, HIGH);
    ecuLine.flushInput();
    ecuLine.begin(10400);
    delay(25);// or 20

    uint8_t data[5] = {0x81, 0x11, 0xF1, 0x81, 0x04};
//    uint8_t data[5] = {129, 17, 241, 129, 4};

    sendSoftSerial(data, 5);
    Serial.println("Waiting ECU response");
    delay(25);


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
    }else{
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
