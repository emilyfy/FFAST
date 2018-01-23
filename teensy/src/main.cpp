#include "vehicle.h"
#include "serial_comms.h"

int main(void) {

    // Initialize USB serial.
    SerialComms ser(115200);

    // Initialize the Vehicle.
    Vehicle vehicle = Vehicle();

    SerialFloat serial_float;
    
    while (1) {
        if( Serial.available() ) {
            switch( ser.read_char() ) {
                case 'v': {
                    float velocity = ser.read_float();
                    if (ser.read_char() == '\n') {
                        vehicle.set_throttle_speed(velocity);
                    }
                } break;

                case 's': {
                    float steering_angle_rads = ser.read_float();
                    if (ser.read_char() == '\n') {
                        vehicle.set_steering_angle(steering_angle_rads);
                    }
                } break;

                case 'e': {
                    if (ser.read_char() == '\n') { vehicle.estop_status = true; }
                } break;

                case 'p': {
                    uint8_t profile_number = ser.read_char();
                    if (ser.read_char() == '\n') { vehicle.run_profile(profile_number); }
                } break;

                default: {
                    // Invalid commands are ignored.
                } break;
            }
        }

        ser.write_char('o');
        ser.write_float(vehicle.get_odom_m());
        ser.write_char('\n');

        ser.write_char('s');
        ser.write_float(vehicle.get_speed_mps());
        ser.write_char('\n');
    }

    return 0;
}
