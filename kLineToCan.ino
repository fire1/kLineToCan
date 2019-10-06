/**
   This program is used to read the ISO 9141-2 OBD port on a vehicle.
   It will read any stored Diagnostic Trouble Codes (DTCs) listed in
   the vehicle's computer and display them to the user.

   A software UART is used to communicate with the vehicle. It
   requires a few external hardware components to operate on the
   bi-directional K-Line used in ISO 9141-2.  The Arduino's hardware
   UART is used to send the stored DTCs as ASCII text to the user.
*/

//NOTE: snprintf_P and PSTR are used extensive together in this program.
//      The combination allows using an snprintf function with an
//      argument string that is stored in FLASH memory without using
//      valuable SRAM space.
#include <Arduino.h>
// Use FLASH memory to store strings instead of SRAM
#include <avr/pgmspace.h>
// (Software USART) AltSoftSerial is used because it can handle non-standard baud rates
#include "lib/header.h"


// Run once after Arduino start-up/reset
void setup() {
    digitalWrite(txPin, HIGH);
    Serial.begin(115200);
    Serial.println(F("Booting ..."));
    delay(3000);
    // Open (hardware) serial port now while we aren't restricted by timing
    Serial.println(F("Starting ..."));
    // Use on-board LED for indication
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, LOW);

    lpgLine.begin(10400);

    // If more than 5 seconds passes between messages then we need to resend the init sequence
    // so use this timer to send 'ping' messages every 4 seconds to keep the connection alive.
    timerKeepAlive.setTimeout(4000);

    // Send Init sequence once to open communications

    sendOpelInit();
}


void loop() {
//    if (ecuLine.available()) {
//        Serial.println(" ECU:  ");
//        Serial.write(ecuLine.read());
//    }

    if (lpgLine.available()) {
        Serial.println(" LPG:  ");
        Serial.write(lpgLine.read());
    }

    if (Serial.available()) {
//        String where = Serial.readStringUntil('=');
//        if (where == F("ecu")) {
            int send = Serial.readStringUntil('\n').toInt();
//            int send = Serial.read();
            ecuLine.write(send);
            Serial.write(send);
            Serial.println(F("ECU-> "));
            Serial.write(send);
            return;
//        }

/*
        if (where == F("lpg")) {
            int send = Serial.read();
            ecuLine.write(send);
            Serial.println(F("LPG-> "));
            Serial.write(send);
            return;
        }

        Serial.println(F("Not found.... "));
*/
    }

}

/*
   Our main program loop will display a menu to the user
*/
void loop_() {
    // Display a menu to select which Mode the user wants to use
    Serial.println(F("MENU - Mode Selection"));
    Serial.println(F("  1) Show Current Data"));
    Serial.println(F("  2) Show Freeze Frame Data"));
    Serial.println(F("  3) Show Diagnostic Trouble Codes"));
    Serial.println(F("  4) Clear Diagnostic Trouble Codes and stored values"));
    Serial.println(F("  5) Test results, oxygen sensor monitoring (non CAN only)"));
    Serial.println(F("  6) Test results, other component/system monitoring "));
    Serial.println(F("  7) Show pending Diagnostic Trouble Codes (detected during current or last driving cycle)"));
    Serial.println(F("  8) Control operation of on-board component/system"));
    Serial.println(F("  9) Request vehicle information"));
    Serial.println(F("  A) Permanent Diagnostic Trouble Codes (DTCs) (Cleared DTCs)"));
    Serial.print(F(" Select Mode (1-9,A): "));
    uint8_t chMode = getSerial();
    // Check and convert user input from ASCII
    if ((chMode >= '0') && (chMode <= '9')) {
        chMode -= 0x30;
    } else if ((chMode == 'A') || (chMode == 'a')) {
        chMode = 0x0A;
    } else {
        Serial.println(F("Bad Input"));
        return;
    }
    // Display the input back to the user
    snprintf_P(buffer, BUFLEN, PSTR("%02X"), chMode);
    Serial.println(buffer);

    // Modes where a PID is required
    if ((chMode == 0x01) || (chMode == 0x02) || (chMode == 0x05) || (chMode == 0x08) || (chMode == 0x09)) {
        // Request which PIDs are supported for this Mode
        uint8_t numMsg = sendPid(chMode, 0);
        // Display formatted table of PIDs available
        formatResponse(chMode, 0, numMsg);

        // Now that the user can see supported PIDs, ask for their input
        Serial.print(F("Select PID (00 - FF): "));
        uint8_t chPid;
        // Convert from ASCII to actual hex value
        uint8_t c1 = getSerial();
        if ((c1 >= '0') && (c1 <= '9')) {
            c1 -= 0x30;
        } else if ((c1 >= 'A') && (c1 <= 'F')) {
            c1 -= 0x41;
        } else if ((c1 >= 'a') && (c1 <= 'f')) {
            c1 -= 0x61;
        } else {
            Serial.println(F("Bad Input"));
            return;
        }

        uint8_t c2 = getSerial();
        if ((c2 >= '0') && (c2 <= '9')) {
            c2 -= 0x30;
        } else if ((c2 >= 'A') && (c2 <= 'F')) {
            c2 -= 0x41;
        } else if ((c2 >= 'a') && (c2 <= 'f')) {
            c2 -= 0x61;
        } else if (c2 != '\n') {
            Serial.println(F("Bad Input"));
            return;
        }

        //DEBUG
        Serial.println(c1, HEX);
        Serial.println(c2, HEX);

        // Check if we received 1 or 2 hex digits (convert to nibbles)
        if (c2 == '\n') {
            chPid = c1;
        } else {
            chPid = (c1 << 4) | c2;
        }
        // Display the input back to the user
        snprintf_P(buffer, BUFLEN, PSTR("%02X"), chPid);
        Serial.println(buffer);


        // Send the actual request
        snprintf_P(buffer, BUFLEN, PSTR("Sending Mode %X PID %02X Request"), chMode, chPid);
        Serial.println(buffer);
        numMsg = sendPid(chMode, chPid);
        // Format the response and send to Serial
        formatResponse(chMode, chPid, numMsg);

    }

    // Request DTCs from all ECUs
    if ((chMode == 0x03) || (chMode == 0x07)) {
        uint8_t numMsg = sendPid(chMode);
        //DEBUG
        dumpRawData(numMsg);

        // Display all the DTCs
        Serial.println(F("DTCs:"));
        //DEBUG
        //Serial.print("Num Msg: ");
        //Serial.println(numMsg);
        for (uint8_t curMsg = 0; curMsg < numMsg; curMsg++) {
            // Display the three DTCs
            for (uint8_t k = 0; k < 3; k++) {
                uint16_t dtc = (responseBuffer[curMsg][4 + (2 * k)] << 8) | responseBuffer[curMsg][5 + (2 * k)];
                if (dtc != 0) {  // Don't print blank DTCs
                    snprintf_P(buffer, BUFLEN, PSTR("  %c%04X  (Raw Data: %04X)"), firstDTCChar(dtc), (dtc & 0x3FFF),
                               dtc);
                    Serial.println(buffer);
                }
            }
        }
    }

    // Prompt before erasing DTCs (Mode 4)
    if (chMode == 0x04) {
        Serial.println(F("Delete all stored DTCs (y or n)? "));
        char resp = getSerial();
        if ((resp == 'y') || (resp == 'Y')) {
            sendPid(chMode);
            Serial.println(F("All stored DTCs have been erased"));
        }
    }

    if (chMode == 0x06) {
        Serial.println(F("Mode 6 is used by CAN bus Test Results. Use Mode 5 for ISO 9141 Test Results"));
    }

    if (chMode == 0x0A) {
        Serial.println(F("Mode A is only supported by ISO 15765-4"));
    }

}


/*
  Format the response for the specified Mode and PID combination
*/
//TODO: Read 'source' address [2] and display data for each ECU that responds. Currently, only the first ECU is displayed.
//TODO: Read Mode and PID from response data ([3]=mode, [4]=pid)
//TODO: Allow cont. read on input (remove numMsg and just print something for each response received)
//TODO: Fix all map()ed variables to allow decimal numbers by using math instead of map()
void formatResponse(uint8_t chMode, uint8_t chPid, uint8_t numMsg) {
    //DEBUG
    dumpRawData(numMsg);

    if (chMode == 0x01) {
        // Mode 1 - Show Current Data
        switch (chPid) {
            case 0x00: {
                displaySupportedPids(chMode, chPid, numMsg);
                break;
            }
            case 0x01:  // Monitor status since DTCs cleared
            {
                uint8_t numDTCs = 0;
                for (uint8_t i = 0; i < numMsg; i++) {
                    // Display ECU address
                    snprintf_P(buffer, BUFLEN, PSTR("ECU %2X "), responseBuffer[i][2]);
                    Serial.println(buffer);

                    // Display the MIL indicator status
                    if (responseBuffer[i][5] & 0x80) {
                        Serial.println(F("  MIL: ON"));
                    } else {
                        Serial.println(F("  MIL: OFF"));
                    }

                    // Display the number of DTCs returned
                    Serial.print(F("  DTC_CNT: "));
                    Serial.println(responseBuffer[i][5] & 0x7F, DEC);

                    numDTCs += responseBuffer[i][5] & 0x7F;
                }

                // Print the total number of DTCs
                Serial.print(F("Total DTCs: "));
                Serial.println(numDTCs, DEC);
                break;
            }
            case 0x02:  // DTC that caused required freeze frame data storage
            {
                uint16_t dtc = (responseBuffer[0][5] << 8) | responseBuffer[0][6];
                if (dtc == 0) {
                    Serial.println(F("DTCFRZF: No Freeze Frame Data"));
                } else {
                    snprintf_P(buffer, BUFLEN, PSTR("DTCFRZF: %c%04X"), firstDTCChar(dtc & 0xC0), (dtc & 0x3F));
                    Serial.println(buffer);
                }
                break;
            }
            case 0x03:  // Fuel system 1/2 status
            {
                for (int i = 5; i <= 6; i++) {
                    snprintf_P(buffer, BUFLEN, PSTR("FUELSYS%d "), i - 4);
                    Serial.println(buffer);
                    switch (responseBuffer[0][i]) {
                        case 0x01:
                            Serial.println(F("Open loop - has not yet satisfied conditions to go closed loop"));
                            break;
                        case 0x02:
                            Serial.println(F("Closed loop - using oxygen sensor(s) as feedback for fuel control"));
                            break;
                        case 0x04:
                            Serial.println(
                                    F("Open loop due to driving conditions (e.g. power enrichment, deceleration enleanment)"));
                            break;
                        case 0x08:
                            Serial.println(F("Open loop - due to detected system fault"));
                            break;
                        case 0x10:
                            Serial.println(
                                    F("Closed loop, but fault with at least one oxygen sensor - may be using single oxygen sensor for fuel control"));
                            break;
                    }
                }
                break;
            }
            case 0x04:  // Calculated LOAD Value
            {
                uint32_t load = map(responseBuffer[0][5], 0, 255, -40, 215);
                snprintf_P(buffer, BUFLEN, PSTR("LOAD_PCT: %d%%"), load);
                Serial.println(buffer);
                break;
            }
            case 0x05:  // Engine Coolant Temperature
            {
                uint32_t temp = map(responseBuffer[0][5], 0, 255, 0, 100);
                snprintf_P(buffer, BUFLEN, PSTR("ECT: %d C"), temp);
                Serial.println(buffer);
                break;
            }
            case 0x06: {
                uint32_t fuel = map(responseBuffer[0][5], 0, 255, -100, 99.22);
                snprintf_P(buffer, BUFLEN, PSTR("SHRTFT1: %d%%"), fuel);
                Serial.println(buffer);
                fuel = map(responseBuffer[0][6], 0, 255, -100, 99.22);
                snprintf_P(buffer, BUFLEN, PSTR("SHRTFT3: %d%%"), fuel);
                Serial.println(buffer);
                break;
            }
            case 0x07: {
                uint32_t fuel = map(responseBuffer[0][5], 0, 255, -100, 99.22);
                snprintf_P(buffer, BUFLEN, PSTR("LONGFT1: %d%%"), fuel);
                Serial.println(buffer);
                fuel = map(responseBuffer[0][6], 0, 255, -100, 99.22);
                snprintf_P(buffer, BUFLEN, PSTR("LONGFT3: %d%%"), fuel);
                Serial.println(buffer);
                break;
            }
            case 0x08: {
                uint32_t fuel = map(responseBuffer[0][5], 0, 255, -100, 99.22);
                snprintf_P(buffer, BUFLEN, PSTR("SHRTFT2: %d%%"), fuel);
                Serial.println(buffer);
                fuel = map(responseBuffer[0][6], 0, 255, -100, 99.22);
                snprintf_P(buffer, BUFLEN, PSTR("SHRTFT4: %d%%"), fuel);
                Serial.println(buffer);
                break;
            }
            case 0x09: {
                uint32_t fuel = map(responseBuffer[0][5], 0, 255, -100, 99.22);
                snprintf_P(buffer, BUFLEN, PSTR("LONGFT2: %d%%"), fuel);
                Serial.println(buffer);
                fuel = map(responseBuffer[0][6], 0, 255, -100, 99.22);
                snprintf_P(buffer, BUFLEN, PSTR("LONGFT4: %d%%"), fuel);
                Serial.println(buffer);
                break;
            }
            case 0x0A:  // Fuel Rail Pressure (gauge)
            {
                uint32_t kpa = map(responseBuffer[0][5], 0, 255, 0, 765);
                snprintf_P(buffer, BUFLEN, PSTR("FRP: %d kPa"), kpa);
                Serial.println(buffer);
                break;
            }
            case 0x0B:  // Intake Manifold Absolute Pressure
            {
                uint32_t kpa = map(responseBuffer[0][5], 0, 255, 0, 255);
                snprintf_P(buffer, BUFLEN, PSTR("MAP: %d kPa"), kpa);
                Serial.println(buffer);
                break;
            }
            case 0x0C:  // Engine RPM
            {
                uint32_t rpm = map((responseBuffer[0][5] << 8) | responseBuffer[0][6], 0, 65535, 0, 16383.75);
                snprintf_P(buffer, BUFLEN, PSTR("RPM: %d/min"), rpm);
                Serial.println(buffer);
                break;
            }
            case 0x0D:  // Vehicle Speed Sensor
            {
                uint32_t vss = map(responseBuffer[0][5], 0, 255, 0, 255);
                snprintf_P(buffer, BUFLEN, PSTR("VSS: %d kPa"), vss);
                Serial.println(buffer);
                break;
            }
            case 0x0E:  // Ignition Timing Advance for #1 Cylinder
            {
                uint32_t adv = map(responseBuffer[0][5], 0, 255, -64, 63.5);
                snprintf_P(buffer, BUFLEN, PSTR("SPARKADV: %d deg"), adv);
                Serial.println(buffer);
                break;
            }
            case 0x0F:  // Intake Air Temperature
            {
                uint32_t iat = map(responseBuffer[0][5], 0, 255, -40, 215);
                snprintf_P(buffer, BUFLEN, PSTR("IAT: %d C"), iat);
                Serial.println(buffer);
                break;
            }
            case 0x10:  // Air Flow Rate from Mass Air Flow Sensor
            {
                uint32_t maf = map((responseBuffer[0][5] << 8) | responseBuffer[0][6], 0, 65535, 0, 655.35);
                snprintf_P(buffer, BUFLEN, PSTR("MAF: %d g/s"), maf);
                Serial.println(buffer);
                break;
            }
            case 0x11:  // Absolute Throttle Position
            {
                uint32_t tp = map(responseBuffer[0][5], 0, 255, 0, 100);
                snprintf_P(buffer, BUFLEN, PSTR("TP: %d%%"), tp);
                Serial.println(buffer);
                break;
            }
            case 0x12:  // Commanded Secondary Air Status
            {
                switch (responseBuffer[0][5]) {
                    case 0x01:
                        Serial.println(F("AIR_STAT: UPS (upstream of first catalytic converter)"));
                        break;
                    case 0x02:
                        Serial.println(F("AIR_STAT: DNS (downstream of first catalytic converter inlet)"));
                        break;
                    case 0x04:
                        Serial.println(F("AIR_STAT: OFF (atmosphere / off)"));
                        break;
                }
                break;
            }
            case 0x13:  // Location of Oxygen Sensors
                // PID 0x13 shall only be supported by a given vehicle if PID 0x1D is not supported.
            {
                uint8_t o2loc = responseBuffer[0][5];
                Serial.print(F("O2SLOC: "));
                if (o2loc & 0x01) {
                    Serial.print(F("O2S11 "));
                }
                if (o2loc & 0x02) {
                    Serial.print(F("O2S12 "));
                }
                if (o2loc & 0x04) {
                    Serial.print(F("O2S13 "));
                }
                if (o2loc & 0x08) {
                    Serial.print(F("O2S14 "));
                }
                if (o2loc & 0x10) {
                    Serial.print(F("O2S21 "));
                }
                if (o2loc & 0x20) {
                    Serial.print(F("O2S22 "));
                }
                if (o2loc & 0x40) {
                    Serial.print(F("O2S23 "));
                }
                if (o2loc & 0x80) {
                    Serial.print(F("O2S24 "));
                }
                Serial.println();
                break;
            }
            case 0x14:  // [0x14 - 0x1B] The Bank/Sensor number depends on if PID 0x13 or 0x1D is supported.
            case 0x15:  // Oxygen Sensor Output Voltage
            case 0x16:  // and Short Term Fuel Trim
            case 0x17:  // for each sensor in the
            case 0x18:  // 4 banks (2 sensors each)
            case 0x19:
            case 0x1A:
            case 0x1B: {
                // Get which Bank and Sensor we are reading based on the PID
                char bank;
                char sensor;
                // First check whether PID 0x13 or 0x1D is supported to
                // determine how many banks/sensors are used/supported
                if (gPidSupport & 0x2000) {
                    // PID 0x13 uses 2 banks with 4 sensor positions each
                    if (chPid < 0x18) {
                        bank = 1;
                    } else {
                        bank = 2;
                    }
                    sensor = (chPid % 4) + 1;
                } else { // else i gPidSupport[0] & 0x0008 )
                    // PID 0x1D uses 4 banks with 2 sensor positions each
                    if (chPid < 0x16) {
                        bank = 1;
                    } else if (chPid < 0x18) {
                        bank = 2;
                    } else if (chPid < 0x1A) {
                        bank = 3;
                    } else {
                        bank = 4;
                    }
                    sensor = (chPid % 2) + 1;
                }

                // Display O2 sensor voltage
                // Arduino has a hard time printing double/float so do the math by hand
                // Scale = 0.005V per bit
                uint16_t o2 = responseBuffer[0][5] * 5;   // number in millivolts
                snprintf_P(buffer, BUFLEN, PSTR("O2S%d%d: %d.%dV"), bank, sensor, o2 / 1000, o2 % 1000);
                Serial.println(buffer);

                // Display fuel trim
                uint32_t fuel = map(responseBuffer[0][5], 0, 255, -100, 99.22);
                snprintf_P(buffer, BUFLEN, PSTR("SHRTFT%d%d: %d%%"), bank, sensor, fuel);
                Serial.println(buffer);
                break;
            }
            case 0x1C:  // OBD requirements to which vehicle is designed
            {
                uint8_t obd = responseBuffer[0][5];
                Serial.print(F("OBDSUP: "));
                if (obd == 0x01) {
                    Serial.println(F("OBD II"));
                } else if (obd == 0x02) {
                    Serial.println(F("OBD"));
                } else if (obd == 0x03) {
                    Serial.println(F("OBD and OBD II"));
                } else if (obd == 0x04) {
                    Serial.println(F("OBD I"));
                } else if (obd == 0x05) {
                    Serial.println(F("NO OBD"));
                } else if (obd == 0x06) {
                    Serial.println(F("EOBD"));
                } else if (obd == 0x07) {
                    Serial.println(F("EOBD and OBD II"));
                } else if (obd == 0x08) {
                    Serial.println(F("EOBD and OBD"));
                } else if (obd == 0x09) {
                    Serial.println(F("EOBD, OBD and OBD II"));
                } else if (obd == 0x0A) {
                    Serial.println(F("JOBD"));
                } else if (obd == 0x0B) {
                    Serial.println(F("JOBD and OBD II"));
                } else if (obd == 0x0C) {
                    Serial.println(F("JOBD and EOBD"));
                } else if (obd == 0x0D) {
                    Serial.println(F("JOBD, EOBD, and OBD II"));
                } else if (obd == 0x0E) {
                    Serial.println(F("EURO IV B1"));
                } else if (obd == 0x0F) {
                    Serial.println(F("EURO V B2"));
                } else if (obd == 0x10) {
                    Serial.println(F("EURO C"));
                } else if (obd == 0x11) {
                    Serial.println(F("EMD"));
                } else if (obd <= 0xFA) {
                    Serial.println(F("ISO/SAE reserved"));
                } else {
                    Serial.println(F("SAE J1939 special meaning"));
                }
                break;
            }
            case 0x1D:  // Location of Oxygen Sensors
                // PID 0x1D shall only be supported by a given vehicle if PID 0x13 is not supported.
            {
                uint8_t o2loc = responseBuffer[0][5];
                Serial.print(F("O2SLOC: "));
                if (o2loc & 0x01) {
                    Serial.print(F("O2S11 "));
                }
                if (o2loc & 0x02) {
                    Serial.print(F("O2S12 "));
                }
                if (o2loc & 0x04) {
                    Serial.print(F("O2S21 "));
                }
                if (o2loc & 0x08) {
                    Serial.print(F("O2S22 "));
                }
                if (o2loc & 0x10) {
                    Serial.print(F("O2S31 "));
                }
                if (o2loc & 0x20) {
                    Serial.print(F("O2S32 "));
                }
                if (o2loc & 0x40) {
                    Serial.print(F("O2S41 "));
                }
                if (o2loc & 0x80) {
                    Serial.print(F("O2S42 "));
                }
                Serial.println();
                break;
            }
            case 0x1E:  // Auxiliary Input Status
            {
                if (responseBuffer[0][5] & 0x01) {
                    Serial.println(F("PTO_STAT: ON"));
                } else {
                    Serial.println(F("PTO_STAT: OFF"));
                }
                // Bits 1-7 are Reserved and should be reported as '0'
            }
            case 0x1F:  // Time Since Engine Start
            {
                uint16_t uptime = (responseBuffer[0][5] << 8) | responseBuffer[0][6];
                snprintf_P(buffer, BUFLEN, PSTR("RUNTM: %d sec."), uptime);
                Serial.println(buffer);
                break;
            }
            case 0x20:  // PIDs supported [21 - 40] (same format as PID 0x00)
            {
                displaySupportedPids(chMode, chPid, numMsg);
                break;
            }
            default:    // PIDs 0x84-0xFF ISO/SAE Reserved
                dumpRawData(numMsg);
                break;
        }
    } else if (chMode == 0x02) {
        //TODO: Mode 2 - Show Freeze Frame Data (Not Yet Implemented. Similar to Mode 1)
    } else if (chMode == 0x05) {
        //TODO: Mode 5 - (Non-CAN) Test Results (Not Yet Implemented)
    } else if (chMode == 0x08) {
        //TODO: Mode 8 (SMOG Testing?) Probably Won't Be Implemented
    } else if (chMode == 0x09) {
        // Mode 9 - Vehicle Information
        switch (chPid) {
            case 0x00: {
                displaySupportedPids(chMode, chPid, numMsg);
                break;
            }
            case 0x01:  // MessageCount VIN (Mode 9 PID 2)
            {
                snprintf_P(buffer, BUFLEN, PSTR("MC_VIN: %d"), responseBuffer[0][5]);
                Serial.println(buffer);
                break;
            }
            case 0x02:  // Vehicle Identification Number (VIN)
            {
                if (numMsg == 5) {
                    char vin[18]; // 17 ASCII chars + '\0'
                    // Nested loops were hard to read, so just copy the chars over one by one
                    vin[0] = responseBuffer[0][8];
                    vin[1] = responseBuffer[1][5];
                    vin[2] = responseBuffer[1][6];
                    vin[3] = responseBuffer[1][7];
                    vin[4] = responseBuffer[1][8];
                    vin[5] = responseBuffer[2][5];
                    vin[6] = responseBuffer[2][6];
                    vin[7] = responseBuffer[2][7];
                    vin[8] = responseBuffer[2][8];
                    vin[9] = responseBuffer[3][5];
                    vin[10] = responseBuffer[3][6];
                    vin[11] = responseBuffer[3][7];
                    vin[12] = responseBuffer[3][8];
                    vin[13] = responseBuffer[4][5];
                    vin[14] = responseBuffer[4][6];
                    vin[15] = responseBuffer[4][7];
                    vin[16] = responseBuffer[4][8];
                    vin[17] = '\0';
                    // Print the VIN "string"
                    snprintf_P(buffer, BUFLEN, PSTR("VIN: %s"), vin);
                    Serial.println(buffer);
                } else {
                    snprintf_P(buffer, BUFLEN, PSTR("ERROR: Expecting 5 messages and received %d"), numMsg);
                    Serial.println(buffer);
                }
                break;
            }
            case 0x03:  // MessageCount CALID
            {
                snprintf_P(buffer, BUFLEN, PSTR("MC_CALID: %d"), responseBuffer[0][5]);
                Serial.println(buffer);
                if ((responseBuffer[0][5] % 4) != 0) {
                    Serial.println(F("ERROR: MC_CALID was not a multiple of 4"));
                }
                break;
            }
            case 0x04:  // Calibration Identifications
            {
                for (int calNum = 0; calNum < numMsg; calNum += 4) {
                    char calid[17]; // 16 ASCII chars + '\0'
                    // Nested loops were hard to read, so just copy the chars over one by one
                    calid[0] = responseBuffer[calNum][5];
                    calid[1] = responseBuffer[calNum][6];
                    calid[2] = responseBuffer[calNum][7];
                    calid[3] = responseBuffer[calNum][8];
                    calid[4] = responseBuffer[calNum + 1][5];
                    calid[5] = responseBuffer[calNum + 1][6];
                    calid[6] = responseBuffer[calNum + 1][7];
                    calid[7] = responseBuffer[calNum + 1][8];
                    calid[8] = responseBuffer[calNum + 2][5];
                    calid[9] = responseBuffer[calNum + 2][6];
                    calid[10] = responseBuffer[calNum + 2][7];
                    calid[11] = responseBuffer[calNum + 2][8];
                    calid[12] = responseBuffer[calNum + 3][5];
                    calid[13] = responseBuffer[calNum + 3][6];
                    calid[14] = responseBuffer[calNum + 3][7];
                    calid[15] = responseBuffer[calNum + 3][8];
                    calid[16] = '\0';
                    // Print the CALID "string"
                    snprintf_P(buffer, BUFLEN, PSTR("CALID: %s"), calid);
                    Serial.println(buffer);
                }
                break;
            }
            case 0x05:  // MessageCount CVN
            {
                snprintf_P(buffer, BUFLEN, PSTR("MC_CVN: %d"), responseBuffer[0][5]);
                Serial.println(buffer);
                break;
            }
            case 0x06:  // Calibration Verification Numbers
            {
                for (int numCvn = 0; numCvn < numMsg; numCvn++) {
                    // CVN uses 4 bytes of HEX data
                    snprintf_P(buffer, BUFLEN, PSTR("CVN: %X%X%X%X"), responseBuffer[numCvn][5],
                               responseBuffer[numCvn][6], responseBuffer[numCvn][7], responseBuffer[numCvn][8]);
                    Serial.println(buffer);
                }
                break;
            }
            case 0x07:  // MessageCount IPT
            {
                snprintf_P(buffer, BUFLEN, PSTR("MC_IPT: %d"), responseBuffer[0][5]);
                Serial.println(buffer);
                break;
            }
            case 0x08:  // In-use Performance Tracking
            {
                // We are expecting either 16 or 20 messages (4 data bytes each)
                // in the specific order definded in SAE 1979/ISO 9141-2
                Serial.println(F("IPT: "));
                for (int ipt = 0; ipt < numMsg; ipt++) {
                    uint16_t counts = (responseBuffer[0][5] << 8) | responseBuffer[0][6];
                    // Format the data depending on the message number we are on
                    if (ipt == 0) {
                        snprintf_P(buffer, BUFLEN, PSTR("OBDCOND: %d cnts"), counts);
                    } else if (ipt == 1) {
                        snprintf_P(buffer, BUFLEN, PSTR("IGNCNTR: %d cnts"), counts);
                    } else if (ipt == 2) {
                        snprintf_P(buffer, BUFLEN, PSTR("CATCOMP1: %d cnts"), counts);
                    } else if (ipt == 3) {
                        snprintf_P(buffer, BUFLEN, PSTR("CATCOND1: %d cnts"), counts);
                    } else if (ipt == 4) {
                        snprintf_P(buffer, BUFLEN, PSTR("CATCOMP2: %d cnts"), counts);
                    } else if (ipt == 5) {
                        snprintf_P(buffer, BUFLEN, PSTR("CATCOND2: %d cnts"), counts);
                    } else if (ipt == 6) {
                        snprintf_P(buffer, BUFLEN, PSTR("O2SCOMP1: %d cnts"), counts);
                    } else if (ipt == 7) {
                        snprintf_P(buffer, BUFLEN, PSTR("O2SCOND1: %d cnts"), counts);
                    } else if (ipt == 8) {
                        snprintf_P(buffer, BUFLEN, PSTR("O2SCOMP2: %d cnts"), counts);
                    } else if (ipt == 9) {
                        snprintf_P(buffer, BUFLEN, PSTR("O2SCOND2: %d cnts"), counts);
                    } else if (ipt == 10) {
                        snprintf_P(buffer, BUFLEN, PSTR("EGRCOMP: %d cnts"), counts);
                    } else if (ipt == 11) {
                        snprintf_P(buffer, BUFLEN, PSTR("EGRCOND: %d cnts"), counts);
                    } else if (ipt == 12) {
                        snprintf_P(buffer, BUFLEN, PSTR("AIRCOMP: %d cnts"), counts);
                    } else if (ipt == 13) {
                        snprintf_P(buffer, BUFLEN, PSTR("AIRCOND: %d cnts"), counts);
                    } else if (ipt == 14) {
                        snprintf_P(buffer, BUFLEN, PSTR("EVAPCOMP: %d cnts"), counts);
                    } else if (ipt == 15) {
                        snprintf_P(buffer, BUFLEN, PSTR("EVAPCOND: %d cnts"), counts);
                    } else if (ipt == 16) {
                        snprintf_P(buffer, BUFLEN, PSTR("SO2SCOMP1: %d cnts"), counts);
                    } else if (ipt == 17) {
                        snprintf_P(buffer, BUFLEN, PSTR("SO2SCOND1: %d cnts"), counts);
                    } else if (ipt == 18) {
                        snprintf_P(buffer, BUFLEN, PSTR("SO2SCOMP2: %d cnts"), counts);
                    } else if (ipt == 19) {
                        snprintf_P(buffer, BUFLEN, PSTR("SO2SCOND2: %d cnts"), counts);
                    } /* else ERROR */
                    // Print the data we formatted
                    Serial.println(buffer);
                }
                break;
            }
            case 0x09:  // MessageCount ECUNAME
            {
                // For ISO 9141-2 the message count in the response should be 0x05
                snprintf_P(buffer, BUFLEN, PSTR("MC_ECUNM: %d"), responseBuffer[0][5]);
                Serial.println(buffer);
                break;
            }
            case 0x0A:  // ECUNAME
            {
                for (int ecuNum = 0; ecuNum < numMsg; ecuNum += 5) {
                    char ecuAbbrv[5]; // 4 ASCII chars + '\0'
                    char ecuFull[16]; // 15 ASCII chars + '\0'
                    // Nested loops were hard to read, so just copy the chars over one by one
                    ecuAbbrv[0] = responseBuffer[ecuNum][5];
                    ecuAbbrv[1] = responseBuffer[ecuNum][6];
                    ecuAbbrv[2] = responseBuffer[ecuNum][7];
                    ecuAbbrv[3] = responseBuffer[ecuNum][8];
                    ecuAbbrv[4] = '\0';

                    ecuFull[0] = responseBuffer[ecuNum + 1][6];
                    ecuFull[1] = responseBuffer[ecuNum + 1][7];
                    ecuFull[2] = responseBuffer[ecuNum + 1][8];
                    ecuFull[3] = responseBuffer[ecuNum + 2][5];
                    ecuFull[4] = responseBuffer[ecuNum + 2][6];
                    ecuFull[5] = responseBuffer[ecuNum + 2][7];
                    ecuFull[6] = responseBuffer[ecuNum + 2][8];
                    ecuFull[7] = responseBuffer[ecuNum + 3][5];
                    ecuFull[8] = responseBuffer[ecuNum + 3][6];
                    ecuFull[9] = responseBuffer[ecuNum + 3][7];
                    ecuFull[10] = responseBuffer[ecuNum + 3][8];
                    ecuFull[11] = responseBuffer[ecuNum + 4][5];
                    ecuFull[12] = responseBuffer[ecuNum + 4][6];
                    ecuFull[13] = responseBuffer[ecuNum + 4][7];
                    ecuFull[14] = responseBuffer[ecuNum + 4][8];
                    ecuFull[15] = '\0';
                    // Print the CALID "string"
                    snprintf_P(buffer, BUFLEN, PSTR("ECU: %s"), ecuAbbrv);
                    Serial.println(buffer);
                    snprintf_P(buffer, BUFLEN, PSTR("ECUNAME: %s"), ecuFull);
                    Serial.println(buffer);
                }
                break;
            }
            case 0x0B:  // In-use Performance Tracking
            {
                // We are expecting either 16 (4 data bytes each)
                // in the specific order definded in SAE 1979/ISO 9141-2
                Serial.println(F("IPT: "));
                for (int ipt = 0; ipt < numMsg; ipt++) {
                    uint16_t counts = (responseBuffer[0][5] << 8) | responseBuffer[0][6];
                    // Format the data depending on the message number we are on
                    if (ipt == 0) {
                        snprintf_P(buffer, BUFLEN, PSTR("OBDCOND: %d cnts"), counts);
                    } else if (ipt == 1) {
                        snprintf_P(buffer, BUFLEN, PSTR("IGNCNTR: %d cnts"), counts);
                    } else if (ipt == 2) {
                        snprintf_P(buffer, BUFLEN, PSTR("HCCATCOMP: %d cnts"), counts);
                    } else if (ipt == 3) {
                        snprintf_P(buffer, BUFLEN, PSTR("HCCATCOND: %d cnts"), counts);
                    } else if (ipt == 4) {
                        snprintf_P(buffer, BUFLEN, PSTR("NCATCOMP: %d cnts"), counts);
                    } else if (ipt == 5) {
                        snprintf_P(buffer, BUFLEN, PSTR("NCATCOND: %d cnts"), counts);
                    } else if (ipt == 6) {
                        snprintf_P(buffer, BUFLEN, PSTR("NADSCOMP: %d cnts"), counts);
                    } else if (ipt == 7) {
                        snprintf_P(buffer, BUFLEN, PSTR("NADSCOND: %d cnts"), counts);
                    } else if (ipt == 8) {
                        snprintf_P(buffer, BUFLEN, PSTR("PMCOMP: %d cnts"), counts);
                    } else if (ipt == 9) {
                        snprintf_P(buffer, BUFLEN, PSTR("PMCOND: %d cnts"), counts);
                    } else if (ipt == 10) {
                        snprintf_P(buffer, BUFLEN, PSTR("EGSCOMP: %d cnts"), counts);
                    } else if (ipt == 11) {
                        snprintf_P(buffer, BUFLEN, PSTR("EGSCOND: %d cnts"), counts);
                    } else if (ipt == 12) {
                        snprintf_P(buffer, BUFLEN, PSTR("EGRCOMP: %d cnts"), counts);
                    } else if (ipt == 13) {
                        snprintf_P(buffer, BUFLEN, PSTR("EGRCOND: %d cnts"), counts);
                    } else if (ipt == 14) {
                        snprintf_P(buffer, BUFLEN, PSTR("BPCOMP: %d cnts"), counts);
                    } else if (ipt == 15) {
                        snprintf_P(buffer, BUFLEN, PSTR("BPCOND: %d cnts"), counts);
                    } /* else ERROR */
                    // Print the data we formatted
                    Serial.println(buffer);
                }
                break;
            }
            default:    // PIDs 0x0C-0xFF ISO/SAE Reserved
                dumpRawData(numMsg);
                break;
        }
    }

}

void dumpRawData(uint8_t numMsg) {
    Serial.print("Raw Data: ");
    for (uint8_t i = 0; i < numMsg; i++) {
        for (uint8_t j = 0; j < 11; j++) {
            snprintf(buffer, BUFLEN, "%02X ", responseBuffer[i][j]);
            Serial.print(buffer);
        }
        Serial.println();
    }
}


/*
   Get the first character of the DTC.  It is determined by the first
   two bits in the first byte of the DTC returned in a Mode 3 message.
*/
uint8_t firstDTCChar(uint16_t dtc) {
    // Get the first two bits of the high byte
    uint8_t hidtc = highByte(dtc & 0xC000);
    switch (hidtc) {
        case 0x00:
            return 'P';
        case 0x40:
            return 'C';
        case 0x80:
            return 'B';
        case 0xC0:
            return 'U';
    }
    // ERROR - Bad input
    return 0;
}


/*
   Function to display a list of PIDs which may or may not be supported
   chPid determines which PIDs are displayed (e.g. PID 0x20 will display 0x21 through 0x40)
*/
void displaySupportedPids(uint8_t chMode, uint8_t chPid, uint8_t numEcu) {
    gPidSupport = 0;     // Reset global PID support
    // Create a list of globally supported PIDs supported by any ECU in the system
    for (uint8_t curEcu = 0; curEcu < numEcu; curEcu++) {
        // Store the PIDs that are supported for this specific ECU packed into 4 bytes
        uint32_t ecuPidSupport = (responseBuffer[curEcu][5] << 24) | (responseBuffer[curEcu][6] << 16) |
                                 (responseBuffer[curEcu][7] << 8) | responseBuffer[curEcu][8];
        // OR support with the list of globally supported PIDs for this system
        gPidSupport = gPidSupport | ecuPidSupport;

        //DEBUG: Print which PIDs are supported for this ECU
        //snprintf(buffer, BUFLEN, "ECU %2X: %8X", responseBuffer[ecuNum][2], ecuPidSupport);
        //Serial.println(buffer);
    }

    // Display a second Menu with available PIDs
    snprintf_P(buffer, BUFLEN, PSTR("Mode %X PIDs Supported:"), chMode);
    Serial.println(buffer);

    // Show 0x20 PIDs in 2 rows. First PID determined by chPid (e.g. chPid + 1)
    for (uint8_t row = 0; row < 2; row++) {
        Serial.print(F("  "));
        for (uint8_t col = 1; col <= 16; col++) {
            snprintf_P(buffer, BUFLEN, PSTR("%02X "), (16 * row) + col + chPid);
            Serial.print(buffer);
        }
        // Move below the numbers to print support (Y, N, or ?)
        Serial.println();
        Serial.print(F("  "));

        // Now print if the previous PIDs are supported or not
        for (uint8_t bitNum = (16 * row); bitNum < (16 * row) + 16; bitNum++) {
            // Invert the bits to get supported PIDs in numerical order
            if ((gPidSupport >> (31 - bitNum)) & 0x01) {
                Serial.print(F(" Y "));
            } else {
                // If there was no response then we don't know if there
                // is support for PIDs with 0s. If there was a reply
                // then 0s in gPidSupport means that PID is unsupported.
                if (numEcu > 0) {
                    Serial.print(F(" N "));
                } else {
                    Serial.print(F(" ? "));
                }
            }
        }
        Serial.println();

    }

    // Print the number of ECUs that responded to the request
    Serial.print(F("Number of responding ECUs: "));
    Serial.println(numEcu, DEC);

}