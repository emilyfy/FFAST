#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include "jetson_gpio/jetsonGPIO.h"

#define ESTOP_PIN pin31

int main(int argc, char **argv)
{
    ros::init(argc, argv, "estop_listener");
    ros::NodeHandle nh;

    ros::Publisher estop_pub = nh.advertise<std_msgs::Bool>("/commands/estop",1);
    ros::Publisher brake_pub = nh.advertise<std_msgs::Float64>("/commands/motor/brake",1);

    jetsonTX2GPIONumber estopPin = ESTOP_PIN;
    gpioExport(estopPin);
    gpioSetDirection(estopPin, inputPin);

    ros::Rate r(10);

    unsigned int pin_val, prev_pin_val;

    while (ros::ok())
    {
        gpioGetValue(estopPin, &pin_val);
        if (pin_val==high && prev_pin_val==low) {
            ROS_INFO("ESTOP button pressed.");
            
            // brake
            std_msgs::Float64 brake_msg;
            brake_msg.data = 2.0;
            brake_pub.publish(brake_msg);

            // publish estop
            std_msgs::Bool estop_msg;
            estop_msg.data = true;
            estop_pub.publish(estop_msg);
        }
        else if (pin_val==low && prev_pin_val==high) {
            std_msgs::Bool estop_msg;
            estop_msg.data = false;
            estop_pub.publish(estop_msg);
        }
        prev_pin_val = pin_val;
        r.sleep();
    }

    gpioUnexport(estopPin);

    return 0;
}
