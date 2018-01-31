#include <stdio.h>
#include <ros/ros.h>
#include <teensy/Command.h>

int
main(int argc, char ** argv)
{
    ros::init(argc, argv, "send_cmd");
    ros::NodeHandle nh;
    ros::Publisher pub_cmd = nh.advertise<teensy::Command>("/teensy/command", 10);
    teensy::Command cmd;
    char str[32];
    char cmd_c;
    float cmd_val;

    cmd_c;
    cmd_val;
    
    while (ros::ok())
    {
        printf("Enter command (v or s) followed by value (m/s or degrees respectively)\n");
        printf("or enter q to quit: \n");
        scanf("%s", str);
        sscanf(str, "%c%f", &cmd_c, &cmd_val);
        
        if (cmd_c=='v') {
            cmd.vel_mps = cmd_val;
            printf("Velocity set to %f m/s\n", cmd_val);
        } else if (cmd_c=='s') {
            cmd.steering_rad = cmd_val*3.1416/180.0;
            printf("Steering angle set to %f degrees\n", cmd_val);
        }
        else if (cmd_c=='q') {
            cmd.vel_mps = 0;
            cmd.steering_rad = 0;
            pub_cmd.publish(cmd);
            break;
        }

        pub_cmd.publish(cmd);
        
    }

    return 0;
}
