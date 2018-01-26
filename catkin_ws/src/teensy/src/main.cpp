#include "Arduino.h"

#define LED_PIN 13

int main(void) {
    pinMode(LED_PIN, OUTPUT);
    while(1) {
        digitalWrite(LED_PIN, !digitalRead(LED_PIN));
        delay(1000);
    }

    return 0;
}
