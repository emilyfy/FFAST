#include "SerialComms/serial_comms.h"
#include "SerialComms/TinyPacks.h"
#include "SerialComms/cobs.h"

#include <math.h>
#include <stddef.h>

#define MAX_PACKED_DATA 256
#define MAX_COBS_PACKED_DATA 257

void sync_cobs(int fd) {
    size_t bytes = 0;
    uint8_t c;

    while(1) {
        bytes = read(fd, &c, 1);
        if( bytes > 0 && c == 0 ) { break; }
    }
    return;
}

int read_cobs_packet(int fd, uint8_t* output, size_t max_length) {
    size_t  i = 0;
    size_t bytes=0;
    uint8_t c;
    uint8_t cobs_packet[MAX_COBS_PACKED_DATA];

    while(1) {
        bytes = read(fd, &c, 1);
        if( bytes > 0 ) {
            cobs_packet[i] = c;
            i++;
        }
        if( c == 0 ) { break; }
        if( i >= max_length ) { return -1; }
    }

    cobs_decode(cobs_packet, i, output);

    return i;
}

int main(int argc, char ** argv) {

    char device_filename[] = "/dev/teensy";

    int serial_fd = connect_to_serial_port(device_filename);
    
    PackWriter writer;
    PackReader reader;
    uint8_t  packed_data[MAX_PACKED_DATA];
    uint8_t  cobs_packed_data[MAX_COBS_PACKED_DATA];
    int packed_data_length, cobs_packed_data_length;

    // Sync to the end of a packet.
    sync_cobs(serial_fd);
    printf("Done\n");

    bool estop_status = false;
    int64_t odom;
    float vel;

    int count = 0;
    while(1) {
        // Read data from Teensy.
        packed_data_length = read_cobs_packet(serial_fd, packed_data, sizeof(packed_data));
        if( packed_data_length > 0 ) {
            reader.setBuffer(packed_data, sizeof(packed_data));
            reader.next();
            if( reader.openMap() ) {
                while( reader.next() ) {
                    if( reader.match("estop") )     estop_status = reader.getBoolean();
                    else if( reader.match("odom") ) odom = reader.getReal();
                    else if( reader.match("vel") )  vel  = reader.getReal();
                    else reader.next();
                }
                reader.close();

                printf("%d %ld %f\n", estop_status, odom, vel);
            }
        }

        // Send data to Teensy.
        static bool estop = false;
        writer.setBuffer(packed_data, sizeof(packed_data));
        writer.openMap();
        writer.putString("estop");
        writer.putBoolean(estop);
        writer.putString("vel");
        writer.putReal(1.2345);
        writer.putString("steer");
        writer.putReal(2.345);
        writer.putString("prof");
        writer.putInteger(1);
        writer.close();
        packed_data_length = writer.getOffset();

        size_t cobs_packed_data_length = cobs_encode(packed_data, packed_data_length, cobs_packed_data);
        cobs_packed_data[cobs_packed_data_length] = 0;
        write(serial_fd, cobs_packed_data, cobs_packed_data_length+1);

        estop = !estop;
        sleep(0.1);
    }

    //lcm_destroy(lcm);
    return 0;
}
