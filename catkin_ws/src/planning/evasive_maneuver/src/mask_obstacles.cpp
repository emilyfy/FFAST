#include <stdio.h>
#include <ros/ros.h>
#include <obstacle_detector/Obstacles.h>

ros::Publisher pub_obs;

void obsCb(const obstacle_detector::Obstacles::ConstPtr& msg)
{
    static obstacle_detector::Obstacles obs;
    obs.header.frame_id = "map";
    obs.circles.resize(1);

    int index;
    double nearest_dist = 100;
    for (int i=0; i<msg->circles.size(); i++) {
        double dist = fabs( msg->circles[i].center.y );
        if (dist < nearest_dist) {
            nearest_dist = dist;
            index = i;
        }
    }

    if ( msg->circles.size() > 0 && nearest_dist < 0.5) {
        obs.header.stamp = ros::Time::now();
        obs.circles[0].center.x = msg->circles[index].center.x;
        obs.circles[0].center.y = msg->circles[index].center.y;
        obs.circles[0].velocity.x = msg->circles[index].velocity.x;
        obs.circles[0].velocity.y = msg->circles[index].velocity.y;
        obs.circles[0].true_radius = msg->circles[index].true_radius;
        obs.circles[0].radius = msg->circles[index].radius;

        pub_obs.publish(obs);
    }
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "mask_obstacle");
    ros::NodeHandle nh;
    
    pub_obs = nh.advertise<obstacle_detector::Obstacles>("/masked_obstacles", 1);
    ros::Subscriber sub_obs = nh.subscribe("/tracked_obstacles",1,obsCb);

    ros::spin();

    return 0;
}