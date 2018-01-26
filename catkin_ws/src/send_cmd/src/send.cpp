#include <stdio.h>
#include <ros/ros.h>
#include <teensy/Command.h>

int
main(int argc, char ** argv)
{
    ros::init(argc, argv, "send_cmd");
    ros::NodeHandle nh;
    ros::Publisher pub_cmd = nh.advertise<teensy::Command>("/Teensy/command", 10);
    teensy::Command cmd;
    char str[32];
    char cmd_c;
    float cmd_val;

    cmd_c = 'v';
    cmd_val = -50.0;
    
    while (ros::ok())
    {
        //printf("Enter command (v or s) followed by value (m/s or degrees respectively): \n");
        //scanf("%s", str);
        //sscanf(str, "%c%f", &cmd_c, &cmd_val);

        cmd_val += 0.01;
        if (cmd_val>=50) { cmd_val = -50.0; }
        
        if (cmd_c=='v') {
            cmd.vel_mps = cmd_val;
            printf("Velocity set to %f m/s\n", cmd_val);
        } else if (cmd_c=='s') {
            cmd.steering_rad = cmd_val*3.1416/180.0;
            printf("Steering angle set to %f degrees\n", cmd_val);
        }

        pub_cmd.publish(cmd);
        
    }

    return 0;
}
