#include <stdio.h>
#include <ros/ros.h>
#include <obstacle_detector/Obstacles.h>
#include <nav_msgs/Odometry.h>

nav_msgs::Odometry::ConstPtr state;

void stateCb(const nav_msgs::Odometry::ConstPtr& msg)
{
    state = msg;
}

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "put_obstacle");
    ros::NodeHandle nh;
    
    ros::Publisher pub_obs = nh.advertise<obstacle_detector::Obstacles>("/tracked_obstacles", 10);
    ros::Subscriber sub_state = nh.subscribe("/ekf_localization/odom",1,stateCb);
    
    obstacle_detector::Obstacles obs;
    obs.header.frame_id = "map";
    obs.circles.resize(1);
    obs.circles[0].true_radius = 0.1;
    obs.circles[0].radius = 0.2;

    char str[28];
    float pos_x, pos_y, vel_x, vel_y;
    int valid = 0;

    while (ros::ok() && valid != 4)
    {
        printf("Enter syntax: pos_x,pos_y,vel_x,vel_y\n");
        scanf("%s", str);
        valid = sscanf(str, "%f,%f,%f,%f", &pos_x, &pos_y, &vel_x, &vel_y);
        
    }

    while (ros::ok() && !state)
        ros::spinOnce();

    double x = state->pose.pose.position.x + pos_x;
    double y = vel_y > 0 ? state->pose.pose.position.y - fabs(pos_y) : state->pose.pose.position.y + fabs(pos_y);

    ROS_INFO("putting obstacle at [%f, %f] with velocity [%f, %f].", x, y, vel_x, vel_y);
    obs.circles[0].center.x = x;
    obs.circles[0].center.y = y;
    obs.circles[0].velocity.x = vel_x;
    obs.circles[0].velocity.y = vel_y;

    double dt = 0.02;

    ros::Rate r(1/dt);
    while (ros::ok() && fabs(state->pose.pose.position.y-obs.circles[0].center.y) <= fabs(pos_y))
    {
        obs.circles[0].center.y += vel_y*dt;
        obs.circles[0].center.x += vel_x*dt;
        obs.header.stamp = ros::Time::now();
        pub_obs.publish(obs);

        r.sleep();
    }

    return 0;
}