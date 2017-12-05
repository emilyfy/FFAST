#include "vehicle.h"

void setup(void) {

    // Initialize USB serial.
    Serial.begin(115200); 

    // Initialize the Vehicle.
    Vehicle::Instantiate(); 
}

int main(void) {
    setup();

    int dir = 1;
    float angle_rad = 0;
    while(1) {
       Vehicle::Instance()->set_steering_angle(angle_rad);
       angle_rad += dir*M_PI/20;
       if (angle_rad > MAX_STEERING_ANGLE_RAD) {
            dir *= -1;
       }
       if (angle_rad < -MAX_STEERING_ANGLE_RAD) {
            dir *= -1;
       }
       delay(100);
    }
}
