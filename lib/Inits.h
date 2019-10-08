//
// Created by Admin on 9/14/2019.
//

#ifndef KLINECAN_INITS_H
#define KLINECAN_INITS_H

#include <Arduino.h>
#include "header.h"

/*
   Send the 5-baud Initialization sequence to open communications
*/
void send5BaudInit() {
    // ISO 9141-2 uses a 5 baud initialization
    // Neither Hardware nor Software UARTs can transmit at 5 baud,
    // so we must bit bang the init sequence to the OBD port.
    pinMode(txPin, OUTPUT);   // Set the txPin to idle HIGH
    digitalWrite(txPin, HIGH);
    // Indicator LED
    digitalWrite(LED_BUILTIN, HIGH);
    delay(2000); // Leave Tx HIGH for 2 seconds before starting the actual init routine

    Serial.println(F("Starting Init"));

    // Send 0x33 (LSB first) via K-Line to the OBD port (0x33 is address of OBD)
    digitalWrite(txPin, LOW); // start bit
    digitalWrite(LED_BUILTIN, LOW);
    delay(200);               // 5 baud means 200ms per bit
    digitalWrite(txPin, HIGH); //11
    digitalWrite(LED_BUILTIN, HIGH);
    delay(400);
    digitalWrite(txPin, LOW); //00
    digitalWrite(LED_BUILTIN, LOW);
    delay(400);
    digitalWrite(txPin, HIGH); //11
    digitalWrite(LED_BUILTIN, HIGH);
    delay(400);
    digitalWrite(txPin, LOW); //00
    digitalWrite(LED_BUILTIN, LOW);
    delay(400);
    digitalWrite(txPin, HIGH); // stop bit
    digitalWrite(LED_BUILTIN, HIGH);
    delay(200);

    bool bInitError = false;  // If there is an error, set bInitError to true
    String strError = "";     // and store the error description in strError.
    bool iso9141 = false;     // Will either be ISO 9141 or ISO 14230
    // Wait for SYNC byte (0x55)
    uint8_t sync;


    if (!getSoftSerial(sync, 300)) {
        //DEBUG
        snprintf_P(buffer, BUFLEN, PSTR("SYNC Byte: %2X"), sync);
        Serial.println(buffer);

        // Continue Init if we got the correct SYNC Byte
        if (sync == 0x55) {
            uint8_t key1;
            uint8_t key2;
            uint8_t invKey2;
            uint8_t invAddress;
            // Get key1
            if (!getSoftSerial(key1, 20)) {
                // Get key2
                if (!getSoftSerial(key2, 20)) {
                    //DEBUG
                    snprintf_P(buffer, BUFLEN, PSTR("KEY1: %X    KEY2: %X"), key1, key2);
                    Serial.println(buffer);

                    delay(25);  // 25ms <= W4 <= 50ms (Time between finished reciept of key2 and start send of inverted key2)
                    //DEBUG
                    invKey2 = ~key2;
                    snprintf_P(buffer, BUFLEN, PSTR("Sending ~KEY2: %X"), invKey2);
                    Serial.println(buffer);

                    //TODO: Use sendSoftSerial instead so we don't need to duplicate the junk read-back
                    ecuLine.write(invKey2);    // Send back inverted key2
                    uint8_t junk;
                    getSoftSerial(junk, 1); // TX and RX share the same line so we recieve back the byte we just sent
                    //DEBUG
                    snprintf_P(buffer, BUFLEN, PSTR("Received ~KEY2: %X"), junk);
                    Serial.println(buffer);

                    if (!getSoftSerial(invAddress, 50)) {
                        //DEBUG
                        snprintf_P(buffer, BUFLEN, PSTR("Inverted Address: %X"), invAddress);
                        Serial.println(buffer);

                        //HACK: Arduino has a BUG where it cannot compare a char with a constant if
                        //      the char is negative (e.g. 0xCC becomes 0xFFCC)
                        //TODO: Check that this is 0xCC. If we use anything other than char Arduino
                        //      uses significantly more SRAM and we are unable to compile for the
                        //      2K available to use in a Pro Mini
                        if (invAddress == 0xCC) {
                            // Check if we are using ISO 9141-2 (otherwise it is ISO 14230)
                            if ((key1 == 0x08) && (key2 == 0x08)) {
                                iso9141 = true;
                                Serial.println(F("Protocol: ISO 9141-2"));
                                Serial.println(F("5-baud Init SUCCESS"));
                            } else {
                                //ERROR: Only ISO 9141-2 is supported
                                bInitError = true;
                                strError = F("ISO 14230-4 is not supported");
                            }
                        } else {    // if( invAddress == 0xCC ) {
                            //ERROR: Unexpected Inverted Address received
                            bInitError = true;
                            strError = F("Bad Inverted Address Byte");
                        }
                    } else {    // if( !getSoftSerial( invAddress, 50 ) ) {
                        //ERROR: invAddress timeout
                        bInitError = true;
                        strError = F("Did not receive invAddress before timeout occured");
                    }
                } else {    // if( !getSoftSerial( key2, 20 ) ) {
                    //ERROR: key2 timeout
                    bInitError = true;
                    strError = F("Did not receive key1 before timeout occured");
                }
            } else {    // if( !getSoftSerial( key1, 20 ) ) {
                //ERROR: key1 timeout
                bInitError = true;
                strError = F("Did not receive key1 before timeout occured");
            }
        } else {    // if( sync == 0x55 ) {
            //ERROR: SYNC Byte is not correct
            bInitError = true;
            strError = F("Bad SYNC Byte");
        }
    } else {    // if( !getSoftSerial( sync, 300 ) ) {
        //ERROR: sync timeout
        bInitError = true;
        strError = F("Did not receive SYNC byte before Timeout occured");
    }

    // Report any errors that occured during Init and enter inf loop
    if (bInitError) {
        Serial.println(F("5-baud Init FAILED"));
        Serial.println(strError);
        Serial.println(F("RESET to Retry"));
        // Go into inf loop to prevent WDT from being setup
        while (true) {}
    }




    // We have (min) 60ms to setup our serial port before the
    // ECU starts sending back the SYNC byte (0x55)
    // Set baud rate for the Software UART defined by ISO 9141-2 (10.4 Kbaud)
    ecuLine.begin(10400);


    // ecuLine.write()

    // The rest of the timing is a horrible mess of nested if statements
    // Should we have used the dreaded 'goto'?
    // Should we have refactored into smaller functions for such a simple timing segment?
    // I don't know.
    // Just try not to get lost in the jungle of if/else below...

    delay(55);  // 55ms <= P3 (wait between messages) <= 5s
    // Init is over. Start reading data from the ECU

    // 55ms has passed since the last message. We now have (5sec - 55ms) to send another
    timerKeepAlive.restart();
}

void sendReq(byte data[], byte length, SoftwareSerial &digiSerial) {
    Serial.println(F("Sending packet... "));
    for (byte x = 0; x < length; x++) {
        //ssmLine.write(data[x]); //Use this to check with ssmLine var not with reference
        ecuLine.write(data[x]);
        //delay(10);
        Serial.println(data[x]);
    }
    Serial.println(F("done sending."));
}


void sendOpelInit() {
    digitalWrite(txPin, LOW); // start bit
    digitalWrite(LED_BUILTIN, LOW);
    delay(25);
    digitalWrite(txPin, HIGH); //11
    digitalWrite(LED_BUILTIN, HIGH);
    ecuLine.begin(10400);
    delay(25);// or 20

    uint8_t data[5] = {129, 17, 241, 129, 4};

    sendSoftSerial(data, 5);
    delay(25);

    Serial.println("Waiting response");

    bool bInitError = false;  // If there is an error, set bInitError to true
    String strError = "";     // and store the error description in strError.
    bool iso9141 = false;     // Will either be ISO 9141 or ISO 14230
    // Wait for SYNC byte (0x55)
    uint8_t syncStart,syncEnd;
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
        uint8_t tries[3] = {241,193,143}; // 0xF1 0xC1 0xC4
        uint8_t sync;
        sendSoftSerial(tries, 3);

    }
    syncEnd = ecuLine.read();
    if(syncEnd == 196){
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
     * */
}

#endif //KLINECAN_INITS_H
