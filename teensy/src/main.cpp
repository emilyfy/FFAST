#include "BNO055/BNO055.h"
#include <Arduino.h>

int main(void)
{
    Serial.begin(115200);
	pinMode(13, OUTPUT);
    digitalWriteFast(13, LOW);

    Adafruit_BNO055 bno = Adafruit_BNO055();

    if(!bno.begin())
    {
        digitalWriteFast(13, HIGH);
        while (1);
    }

    delay(1000);
    bno.setExtCrystalUse(true);

    while(1) {
       imu::Quaternion quat = bno.getQuat();
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
