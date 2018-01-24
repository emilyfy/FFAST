#include "vehicle.h"
#include "SerialComms/TinyPacks.h"
#include "SerialComms/cobs.h"

#include <stddef.h>
#include <Arduino.h>

#define MAX_PACKED_DATA 256
#define MAX_COBS_PACKED_DATA 257

void sync_cobs() {
    uint8_t c;

    while(1) {
        c = Serial.read();
        if( c == 0 ) { break; }
    }
    return;
}

int read_cobs_packet(uint8_t* output, size_t max_length) {

    size_t i=0;
    size_t bytes=0;
    uint8_t c;
    uint8_t cobs_packet[MAX_COBS_PACKED_DATA];

    if( !Serial.available() ) { return -1; }
    
    while(1) {
        if( Serial.available() ) {
            c = Serial.read();
            cobs_packet[i] = c;
            i++;
            if( c == 0 ) { break; }
            if( i >= max_length ) { return -1; }
        }
    }
    cobs_decode(cobs_packet, i, output);
    return i;
}

int main(void) {

    PackWriter writer;
    PackReader reader;

    uint8_t packed_data[MAX_PACKED_DATA];
    uint8_t cobs_packed_data[MAX_PACKED_DATA];
    int packed_data_length, cobs_packed_data_length;

    // Initialize USB serial.
    Serial.begin(115200);

    float vel, steering_angle;
    float odom = 0;
    bool estop_status = false;
    int p_num;
    
    bool first_time = true;
    
    while (1) {
        // Send data to host computer.
        writer.setBuffer(packed_data, MAX_PACKED_DATA);
        writer.openMap();
        writer.putString("estop");
        writer.putBoolean(false);
        writer.putString("odom");
        writer.putReal(odom++); 
        writer.putString("vel");
        writer.putReal(-1.2345);
        writer.close();
        packed_data_length = writer.getOffset();

        //Serial.write(packed_data, packed_data_length);
        size_t cobs_packed_data_length = cobs_encode(packed_data, packed_data_length, cobs_packed_data);
        cobs_packed_data[cobs_packed_data_length] = 0;
        Serial.write(cobs_packed_data, cobs_packed_data_length+1);


        // Read data from host computer.
        if( Serial.available() && first_time ) {
            sync_cobs();
            first_time = false;
        }
        packed_data_length = read_cobs_packet(packed_data, sizeof(packed_data));

        if( packed_data_length > 0 ) {
            reader.setBuffer(packed_data, packed_data_length);
            reader.next();
            if( reader.openMap() ) {
                while( reader.next() ) {
                    if( reader.match("estop") )      estop_status = reader.getBoolean();
                    else if( reader.match("vel") )   vel = reader.getReal();
                    else if( reader.match("steer") ) steering_angle = reader.getReal();
                    else if( reader.match("prof") )  p_num = reader.getInteger();
                    else reader.next();
                }
            }
        }

    }

    return 0;
}
