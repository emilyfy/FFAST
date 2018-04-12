#include <stdio.h>
#include <ros/ros.h>
#include <time.h>
#include <obstacle_detector/Obstacles.h>
#include <nav_msgs/Odometry.h>

ros::Publisher pub_obs;
int obs_speed;
obstacle_detector::Obstacles obs;

void stateCb(const nav_msgs::Odometry::ConstPtr& msg)
{
    static bool set_obs = false;
    
    ros::Time curr_time = ros::Time::now();
    static ros::Time last_time = curr_time;
    double dt = (curr_time-last_time).toSec();
    last_time = curr_time;

    if (obs_speed == 5) {
        if (msg->pose.pose.position.x > 4.5) {
            obs.circles[0].center.x = -1;
            obs.circles[0].velocity.y = 0;
            obs.header.stamp = curr_time;
            pub_obs.publish(obs);
        }
        else if (!set_obs || obs.circles[0].center.y > msg->pose.pose.position.y+0.5 || obs.circles[0].center.x < msg->pose.pose.position.x-0.5 ) {
            obs.circles[0].velocity.y = 1 + ( (rand()%31)/10.0 );
            obs.circles[0].center.x = msg->pose.pose.position.x + 0.2 + ( (rand()%14)/10.0 );
            obs.circles[0].center.y = msg->pose.pose.position.y - 0.5;
            set_obs = true;
            obs.header.stamp = curr_time;
            pub_obs.publish(obs);
        }
        else {
            obs.circles[0].center.x += obs.circles[0].velocity.x*dt;
            obs.circles[0].center.y += obs.circles[0].velocity.y*dt;
            obs.header.stamp = curr_time;
            pub_obs.publish(obs);
        }
    }
    else {
        if (msg->pose.pose.position.x > 2 && !set_obs) {
            obs.circles[0].center.x = msg->pose.pose.position.x+1;
            obs.circles[0].center.y = -((float)obs_speed/msg->twist.twist.linear.x);
            obs.circles[0].velocity.y = obs_speed;
            set_obs = true;
            obs.header.stamp = curr_time;
            pub_obs.publish(obs);
        }
        else if (set_obs && obs.circles[0].center.y >= msg->pose.pose.position.y+3) {
            obs.circles[0].velocity.y = 0;
            obs.header.stamp = curr_time;
            pub_obs.publish(obs);
        }
        else {
            obs.circles[0].center.x += obs.circles[0].velocity.x*dt;
            obs.circles[0].center.y += obs.circles[0].velocity.y*dt;
            obs.header.stamp = curr_time;
            pub_obs.publish(obs);
        }
    }
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "put_obstacle");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    
    if (!pnh.getParam("obs_speed", obs_speed)) {
        ROS_ERROR("Parameter obs_speed required.");
        return 1;
    }
    
    pub_obs = nh.advertise<obstacle_detector::Obstacles>("/masked_obstacles", 10);
    ros::Subscriber sub_state = nh.subscribe("/ekf_localization/odom",1,stateCb);

    obs.header.frame_id = "map";
    obs.circles.resize(1);
    obs.circles[0].true_radius = 0.1;
    obs.circles[0].radius = 0.2;
    obs.circles[0].center.x = -1;
    obs.circles[0].center.y = 0;
    obs.circles[0].velocity.x = 0;
    obs.circles[0].velocity.y = 0;

    srand (time(NULL));
    
    ros::spin();

    return 0;
}