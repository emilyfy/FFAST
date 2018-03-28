#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/LaserScan.h>

#define OBS_DIST_THRESHOLD 0.5
#define ANGLE_OBSERVANCE   3.14

ros::Publisher estop_pub;

void scanCb(const sensor_msgs::LaserScan& msg)
{
    bool stop = false;
    static bool prev_stop = false;

    int start_idx = (fabs(msg.angle_min) + ANGLE_OBSERVANCE/2)/msg.angle_increment;
    int end_idx = msg.ranges.size() - (msg.angle_max - ANGLE_OBSERVANCE/2)/msg.angle_increment;

    int i;
    for (i = start_idx; i<end_idx; i++)
    {
        if (msg.ranges[i] < OBS_DIST_THRESHOLD)
        {
            stop = true;
            break;
        }
    }
    
    if (stop && !prev_stop) {
        ROS_INFO("Detected scan at %f meters at %f degrees. Stopping.", msg.ranges[i], (msg.angle_min+i*msg.angle_increment)/3.14*180);
        std_msgs::Bool estop_msg;
        estop_msg.data = true;
        estop_pub.publish(estop_msg);
        prev_stop = true;
    }

    else if (!stop && prev_stop) {
        ROS_INFO("No more scan detected within %f meters. Releasing.", OBS_DIST_THRESHOLD);
        std_msgs::Bool estop_msg;
        estop_msg.data = false;
        estop_pub.publish(estop_msg);
        prev_stop = false;
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "estop_listener");
    ros::NodeHandle nh;

    estop_pub = nh.advertise<std_msgs::Bool>("/commands/estop",1);
    ros::Subscriber scan_sub = nh.subscribe("scan", 1, &scanCb);

    ros::spin();

    return 0;
}
