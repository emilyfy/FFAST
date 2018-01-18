#include "vehicle.h"


int main(void)
{

    // Initialize USB serial.
    Serial.begin(115200);

    // Initialize the Vehicle.
    Vehicle vehicle = Vehicle();

    int i;
    char c;
    union {
        float data = 0;
        byte tobyte[];
    }serial_float;
    
    while (1) {

        if (Serial.available())
        {
            c = Serial.read();
            if (c == 'v')
            {
                for (i=0;i<sizeof(serial_float);i++) { serial_float.tobyte[i] = Serial.read(); }
                if (Serial.read() == '\n') { vehicle.set_throttle_speed(serial_float.data); }
            }
            else if (c == 's')
            {
                for (i=0;i<sizeof(serial_float);i++) { serial_float.tobyte[i] = Serial.read(); }
                if (Serial.read() == '\n') { vehicle.set_steering_angle(serial_float.data); }
            }
            else if (c == 'e')
            {
                if (Serial.read() == '\n') { vehicle.estop_isr(); }
            }
            else if (c == 'p')
            {
                uint8_t val = Serial.read();
                if (Serial.read() == '\n') { vehicle.set_profile(val); }
            }
        }

        serial_float.data = vehicle.get_odom_m();
        Serial.write('o');
        for (int i=0; i<sizeof(serial_float); i++) { Serial.write(serial_float.tobyte[i]); }
        Serial.write('\n');
        delayMicroseconds(100);

        serial_float.data = vehicle.get_speed_mps();
        Serial.write('s');
        for (int i=0; i<sizeof(serial_float); i++) { Serial.write(serial_float.tobyte[i]); }
        Serial.write('\n');
        delayMicroseconds(100);

    }

    return 0;
}
