//
// Created by Admin on 9/14/2019.
//

#ifndef KLINECAN_INITS_H
#define KLINECAN_INITS_H

#include <Arduino.h>

void initOrdinary(){
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
}


void initOpel(){
    digitalWrite(txPin, LOW); // start bit
    digitalWrite(LED_BUILTIN, LOW);
    delay(25);
    digitalWrite(txPin, HIGH); //11
    digitalWrite(LED_BUILTIN, HIGH);
    delay(20);

}

#endif //KLINECAN_INITS_H
