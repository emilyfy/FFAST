#include "vesc_interface/multiplexer.h"

#include <cmath>
#include <sstream>

#include <ackermann_msgs/AckermannDriveStamped.h>

namespace vesc_interface
{
    template <typename T>
    inline bool getRequiredParam(const ros::NodeHandle& n, std::string name, T& value);

    Multiplexer::Multiplexer(ros::NodeHandle nh, ros::NodeHandle pnh) :
        mode_(0), timeout_(TIMEOUT)
    {
        if (!getRequiredParam(nh, "vehicle/wheelbase", wheelbase_))
            return;
        
        joy_sub_ = nh.subscribe("/commands/joy",10,&Multiplexer::joyCallback,this);
        nav_sub_ = nh.subscribe("/commands/cmd_vel",10,&Multiplexer::navCallback,this);
        kb_sub_ = nh.subscribe("/commands/keyboard",10,&Multiplexer::kbCallback,this);
        
        ackermann_pub_ = nh.advertise<ackermann_msgs::AckermannDriveStamped>("/commands/ackermann", 10);
        pub_timer_ = nh.createTimer(ros::Duration(4*TIMEOUT), &Multiplexer::pubTimerCallback, this);
    }

    void Multiplexer::joyCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg)
    {
        last_cb_time_[JOY_PRIORITY] = msg->header.stamp;
        ros::Time curr_time = ros::Time::now();
        bool publish = true;
        for (int i=3; i>JOY_PRIORITY; --i) {
            if ( (curr_time - last_cb_time_[i]) < timeout_) publish = false;
        }

        if (publish) {
            mode_ = JOY_PRIORITY;
            ackermann_msg_.header.stamp = msg->header.stamp;
            ackermann_msg_.drive.speed = msg->drive.speed;
            ackermann_msg_.drive.steering_angle = msg->drive.steering_angle;
            ackermann_pub_.publish(ackermann_msg_);
        }
    }

    void Multiplexer::kbCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg)
    {
        last_cb_time_[KB_PRIORITY] = msg->header.stamp;
        ros::Time curr_time = ros::Time::now();
        bool publish = true;
        for (int i=3; i>KB_PRIORITY; --i) {
            if ( (curr_time - last_cb_time_[i]) < timeout_) publish = false;
        }

        if (publish) {
            mode_ = KB_PRIORITY;
            ackermann_msg_.header.stamp = msg->header.stamp;
            ackermann_msg_.drive.speed = msg->drive.speed;
            ackermann_msg_.drive.steering_angle = msg->drive.steering_angle;
            ackermann_pub_.publish(ackermann_msg_);
        }
    }

    void Multiplexer::navCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        ros::Time curr_time = ros::Time::now();
        last_cb_time_[NAV_PRIORITY] = curr_time;

        bool publish = true;
        for (int i=3; i>NAV_PRIORITY; --i) {
            if ( (curr_time - last_cb_time_[i]) < timeout_) publish = false;
        }

        if (publish) {
            mode_ = NAV_PRIORITY;
            ackermann_msg_.header.stamp = curr_time;
            ackermann_msg_.drive.speed = msg->linear.x;
            ackermann_msg_.drive.steering_angle = angVelToSteer_(msg->linear.x, msg->angular.z);
            ackermann_pub_.publish(ackermann_msg_);
        }
    }

    void Multiplexer::pubTimerCallback(const ros::TimerEvent& event)
    {
        ros::Time curr_time = ros::Time::now();
        last_cb_time_[DEFAULT_PRIORITY] = curr_time;
        bool publish = true;
        for (int i=3; i>DEFAULT_PRIORITY; --i) {
            if ( (curr_time - last_cb_time_[i]) < timeout_) publish = false;
        }

        if (publish) {
            mode_ = DEFAULT_PRIORITY;
            ackermann_msg_.header.stamp = curr_time;
            ackermann_msg_.drive.speed = 0.0;
            ackermann_msg_.drive.steering_angle = 0.0;
            ackermann_pub_.publish(ackermann_msg_);
        }
    }

    double Multiplexer::angVelToSteer_(double v, double omega) {
        double steering_angle;
        if (v == 0 || omega == 0) {
            steering_angle = 0;
        }
        else {
            double turn_radius = v / omega;
            steering_angle = atan(wheelbase_ / turn_radius);
        }
        return steering_angle;
    }

    template <typename T>
    inline bool getRequiredParam(const ros::NodeHandle& n, std::string name, T& value)
    {
        if (n.getParam(name, value))
            return true;

        ROS_FATAL("Multiplexer: Parameter %s is required.", name.c_str());
        return false;
    }
}