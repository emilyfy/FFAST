#include "BNO055/BNO055.h"
#include "PWMServo/PWMServo.h"
#include <Arduino.h>

// Global Inputs
Adafruit_BNO055 accel = Adafruit_BNO055();

// Global Outputs
PWMServo steering_servo;
PWMServo throttle_servo;

void error_blink(int period_ms) {
    while(1) {
        digitalWriteFast(13, LOW);
        delay(period_ms>>1);
        digitalWriteFast(13, HIGH);
        delay(period_ms>>1);
    }
}

void setup(void) {
    // Turn on LED to indicate aliveness.
	pinMode(13, OUTPUT);
    digitalWriteFast(13, HIGH);

    // Initialize the servo outputs.
    steering_servo.attach(4);
    throttle_servo.attach(3);

    // Initialize USB serial.
    Serial.begin(115200);

    // Initialize accelerometer.
    accel = Adafruit_BNO055();

    if(!accel.begin()) {
        error_blink(1000);
    }

    delay(1000);
    accel.setExtCrystalUse(true);
}

int main(void) {
    setup();

    steering_servo.write(0);
    throttle_servo.write(20);

    while(1) {
       imu::Quaternion quat = accel.getQuat();
       Serial.print("(");
       Serial.print(quat.w(), 4);
       Serial.print(", ");
       Serial.print(quat.x(), 4);
       Serial.print(", ");
       Serial.print(quat.y(), 4);
       Serial.print(", ");
       Serial.print(quat.z(), 4);
       Serial.print(")\n");
       delay(10);
    }
}
