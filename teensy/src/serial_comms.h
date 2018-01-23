#ifndef TEENSY_SERIAL_COMMS_H_
#define TEENSY_SERIAL_COMMS_H_

#include <Arduino.h>

typedef union {
    float     data = 0;
    uint8_t tobyte[sizeof(float)];
} SerialFloat;

class SerialComms {
 public:

    ///< Initialize underlying Serial object with correct baudrate.
    SerialComms(int baudrate) { Serial.begin(baudrate); }

    ///< Pass through the number of bytes available to read.
    int available(void) { return Serial.available(); }

    ///< Read a single character from Serial.
    char read_char(void) { return Serial.read(); }

    ///< Write a single character to Serial.
    void write_char(char byte) { Serial.write(byte); }

    ///< Read in a float. Hope that endianness is correct.
    float read_float(void) {
        SerialFloat serial_float;
        for (size_t i = 0; i < sizeof(serial_float.data); i++) {
            serial_float.tobyte[i] = Serial.read();
        }
        return serial_float.data;
    }

    ///< Write out a float. Hope that endianness is correct.
    void write_float(float flt) {
        SerialFloat serial_float;
        serial_float.data = flt;
        for (size_t i=0; i < sizeof(serial_float.data); i++) {
            Serial.write(serial_float.tobyte[i]);
        }
    }
};

#endif // TEENSY_SERIAL_COMMS_H_

