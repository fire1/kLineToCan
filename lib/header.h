//
// Created by Angel Zaprianov on 2019-09-17.
//

#ifndef KLINECAN_HEADER_H
#define KLINECAN_HEADER_H

#include <SoftwareSerial.h>


SoftwareSerial lpgLine(2, 3); // RX, TX

#include <AltSoftSerial.h>
// (Software Timer) Used to keep the connection alive
#include <RBD_Timer.h>

#ifndef RBD_TIMER_H

#include "../libraries/RBD_Timer/src/RBD_Timer.h

#endif

#ifndef AltSoftSerial_h

#include "../libraries/AltSoftSerial/AltSoftSerial.h"

#endif


const uint8_t pinEcu = A0;
const uint8_t pinLpg = A1;

//#define LED_BUILTIN 13


#define txPin 9
#define rxPin 8
// Must use the same UART pins from AltSoftSerial
AltSoftSerial ecuLine(txPin, rxPin); // Uses Tx = 9, Rx = 8

// The max number of ECUs that will be read from (determines how much RAM will be used)
#define MAX_RESPONSES 15

// Hold a list of globally supported PIDs. A specific PID is supported if
// any ECU connected to the system supports the PID.
// Stored in reverse bit order
//   e.g. PID 0x01 = bit 32, PID 0x02 = bit 31, ... PID 0x20 = bit 1
// For now just store 0x20 PIDs at a time
// TODO: Store all PIDs at once if we can find room in SRAM
uint32_t gPidSupport = 0;


// A buffer to hold all responses to any request messages
// Response messages are always 11 bytes or less:
//   Header1, Header2, Header3 (Source), Data (up to 7 bytes), Checksum
uint8_t responseBuffer[MAX_RESPONSES][11] = {0};


// A buffer to hold strings that will be displayed to the user via UART
#define BUFLEN 64
//TODO: Change to uint8_t once we can find an alternative to Serial.println(buffer)
//      Try Serial.println((char*)buffer)
char buffer[BUFLEN] = {0};


// Used to send a 'ping' request every few seconds to keep the connection alive
RBD::Timer timerKeepAlive;


static boolean getSoftSerial(uint8_t &retVal, uint32_t timeout);
/**
 * A function that will send (Tx) an array of bytes to the Software Serial port
 * @param bytes
 * @param size
 * @param pause
 */
void sendSoftSerial(uint8_t *bytes, unsigned size, uint8_t pause = 5) {
    //DEBUG
    /*
      Serial.println();
      Serial.print("Sending: ");
      for( int i = 0; i < size; i++ ) {
        snprintf(buffer, BUFLEN, "%02X ", bytes[i]);
        Serial.print(buffer);
      }
      Serial.println();
    */

    for (int i = 0; i < size; i++) {
        // Send the next byte
        ecuLine.write(bytes[i]);

        // todo try using ecuLine.flushInput()  instead getSoftSerial
        // TX and RX share the same line so we recieve back the byte we just sent
        uint8_t trash;
        getSoftSerial(trash, 1);
        //TODO: This can be adjusted depending on how long the above
        //      function takes to execute. 5ms <= P4 <= 20ms
        delay(pause); // P4 (Inter-byte spacing)
    }
}

/*
  Send a request to system as a the 'Diagnostic Tool/External Test Equipment'

  @param mode The Mode of the PID
  @param pid The PID to request (or default of 0 if requesting Mode 3 or 4)
  @param ping If true, we won't save any response data in the global responseBuffer
  @return The number of response messages received
*/
uint8_t sendPid(uint8_t mode, uint8_t pid = 0, boolean ping = false) {
    // Flash LED to show we are sending data
    digitalWrite(LED_BUILTIN, LOW);

    // Message format: Header1, Header2, Source Address, Mode, (PID), Checksum   (see SAE J1979)
    uint8_t request[6] = {0x68, 0x6A, 0xF1, mode, 0x00, 0x00};
    uint8_t crc = (0x68 + 0x6A + 0xF1 + mode);

    // Format the request based on which mode we are requesting
    if ((mode == 0x03) || (mode == 0x04)) {
        // Mode 3 and 4 require no PID
        request[4] = crc;
        // request[5] is unused, leave it as 0x00
        sendSoftSerial(request, 5);   // Send the request
    } else {
        // Modes 1, 2, 5, and 9 require a PID
        crc += pid;
        request[4] = pid;
        request[5] = crc;
        sendSoftSerial(request, 6);   // Send the request
    }
    // The ECU(s) will now wait 50ms (P2) to ensure that we are done transmitting before responding
    delay(50);

    // Get all responses and store them in the responseBuffer
    // Start loop to get a response from each ECU that wants to reply
    boolean bMsgTimeout = false;
    uint8_t messageNum = 0;

    while (!bMsgTimeout) {
        boolean bByteTimeout = false;
        uint8_t trash;

        //TODO: Define constants for timeouts
        uint8_t byteNum = 0;
        while (!bByteTimeout) {
            if (ping) {
                bByteTimeout = getSoftSerial(trash, 20); // Store response
            } else {
                bByteTimeout = getSoftSerial(responseBuffer[messageNum][byteNum], 20); // Store response
                if (bByteTimeout) {
                    // Set the current byte to 0 if the timeout occured before it could be written
                    responseBuffer[messageNum][byteNum] = 0;
                }
            }

            byteNum++;
        }


        // Wait for another ECU to respond or for message timeout to occur
        // 55ms must pass before we know that no other ECU will respond
        // The previous message timed out for the last byte recieved so add
        // the byteTimeout to the start time, and wait for messageTimeout
        unsigned long startTime = millis() - 20;

        while (!ecuLine.available()) {
            // Timeout is 55ms
            if ((millis() - startTime) >= 55) {
                bMsgTimeout = true;   // Leave the outter 'while' loop
                break;  // Leave this 'while' loop
            }
        }

        // Fill the rest of the current message with zeros if this is not a ping
        if (!ping) {
            for (int i = byteNum; byteNum < 11; byteNum++) {
                responseBuffer[messageNum][byteNum] = 0;
            }
        }

        // Move to the next message if timeout was BETWEEN 20ms and 55ms
        messageNum++;
    }

    // 55ms has passed since the last message. We now have (5sec - 55ms) to send another
    timerKeepAlive.restart();

    // Flash LED to show we are sending data
    digitalWrite(LED_BUILTIN, HIGH);

    return messageNum;
}


/*
   A function that will wait for data to become available on the
   Hardware Serial port, then return the data.
*/
uint8_t getSerial() {
    while (!Serial.available()) {
        //Check the keep-alive timer while we wait for a byte to arrive
        if (timerKeepAlive.isExpired()) {
            // Send a 'keep-alive' message
            sendPid(1, 0, true);
            timerKeepAlive.restart();
        }
    }
    // Return the (16-bit) int as a single (8-bit) byte
    // We always check for data first with Serial.available()
    // so no need to check for an empty buffer (e.g. 0xFFFF)
    return Serial.read();
}


/*
   A function that will wait for a byte to become available on the
   Software Serial port OR for timeout to occur. If data is available
   before the timeout, then return the data.

   @param retVal The character read on the software serial port
   @param timeout The number of milliseconds to wait for a byte to be read on the port
   @return Did the timeout occur before reading? False: read is good, True: retVal was not filled in time (failed)
*/
boolean     getSoftSerial(uint8_t &retVal, uint32_t timeout) {
    uint32_t start = millis();
    while (!ecuLine.available()) {
        //Wait for a byte to arrive
        if ((millis() - start) > timeout) {
            return true;
        }
    }
    // Return the (16-bit) int as a single (8-bit) byte
    // We always check for data first with Serial.available()
    // so no need to check for an empty buffer (e.g. 0xFFFF)
    retVal = ecuLine.read();
    return false;
}


#endif //KLINECAN_HEADER_H
