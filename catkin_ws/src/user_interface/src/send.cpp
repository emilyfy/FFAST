#include <stdio.h>
#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>

int
main(int argc, char ** argv)
{
    ros::init(argc, argv, "send_cmd");
    ros::NodeHandle nh;
    ros::Publisher pub_cmd = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/commands/keyboard", 10);
    ackermann_msgs::AckermannDriveStamped cmd;
    char str[32];
    char cmd_c;
    float cmd_val;
    
    while (ros::ok())
    {
        printf("Enter command (v or s) followed by value (m/s or degrees respectively)\n");
        printf("or enter q to quit: \n");
        scanf("%s", str);
        sscanf(str, "%c%f", &cmd_c, &cmd_val);
        
        if (cmd_c=='v') {
            cmd.drive.speed = cmd_val;
            printf("Velocity set to %f m/s\n", cmd_val);
        } else if (cmd_c=='s') {
            cmd.drive.steering_angle = cmd_val*3.1416/180.0;
            printf("Steering angle set to %f degrees\n", cmd_val);
        }
        else if (cmd_c=='q') {
            cmd.drive.speed = 0;
            cmd.drive.steering_angle = 0;
            pub_cmd.publish(cmd);
            break;
        }

        cmd.header.stamp = ros::Time::now();
        pub_cmd.publish(cmd);
        
    }

    return 0;
}
