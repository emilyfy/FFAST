#include <Arduino.h>

int main(void) {
    pinMode(13, OUTPUT);
    
    while(1) {
        digitalWrite(13, !digitalRead(13));
        delay(1);
    }
    return 0;
}
