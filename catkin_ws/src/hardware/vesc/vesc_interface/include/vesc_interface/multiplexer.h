#ifndef MULTIPLEXER_H_
#define MULTIPLEXER_H_

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/Twist.h>

#define JOY_PRIORITY     3 // top priority
#define KB_PRIORITY      2
#define NAV_PRIORITY     1
#define DEFAULT_PRIORITY 0
#define TIMEOUT         0.5 // 0.5 seconds

namespace vesc_interface
{

    class Multiplexer
    {
    public:

        Multiplexer(ros::NodeHandle nh, ros::NodeHandle pnh);

    private:
        
        // ROS parameters
        double wheelbase_;
        
        // variables
        int mode_;
        ros::Duration timeout_;
        ros::Time last_cb_time_[4];

        // ROS services
        ros::Publisher ackermann_pub_;
        ros::Subscriber estop_sub_;
        ros::Subscriber joy_sub_;
        ros::Subscriber kb_sub_;
        ros::Subscriber nav_sub_;
        ros::Timer pub_timer_;

        // ROS messages
        ackermann_msgs::AckermannDriveStamped ackermann_msg_;
        
        // ROS callbacks
        void estopCallback(const std_msgs::Bool::ConstPtr& msg);
        void joyCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg);
        void kbCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& msg);
        void navCallback(const geometry_msgs::Twist::ConstPtr& msg);

        // timer callback to publish
        void pubTimerCallback(const ros::TimerEvent& event);

        // convert twist to ackermann
        double angVelToSteer_(double v, double omega);
    };

} // namespace vesc_interface

#endif // MULTIPLEXER_H_
