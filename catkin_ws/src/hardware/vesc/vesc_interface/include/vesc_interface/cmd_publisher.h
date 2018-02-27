#ifndef CMD_PUBLISHER_H_
#define CMD_PUBLISHER_H_

#include <ros/ros.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <std_msgs/Float64.h>

namespace vesc_interface
{

    class CmdPublisher
    {
    public:

        CmdPublisher(ros::NodeHandle nh, ros::NodeHandle pnh);

    private:
        // ROS parameters
        // conversion gain and offset
        double speed_gain_, speed_offset_;
        double steering_gain_, steering_offset_;

        // limitter and smoother
        double max_motor_accel_, motor_smoother_rate_;
        double max_servo_speed_, servo_smoother_rate_;
        double max_speed_chg_, max_pos_chg_;
        bool limit_motor_, limit_servo_;
        bool smooth_motor_, smooth_servo_;

        // variables
        double last_speed_, target_speed_;
        double last_pos_, target_pos_;
        ros::Time last_pub_motor_time_;
        ros::Time last_pub_servo_time_;

        // ROS services
        ros::Publisher motor_pub_;
        ros::Publisher servo_pub_;
        ros::Subscriber ackermann_sub_;
        ros::Timer motor_pub_timer_;
        ros::Timer servo_pub_timer_;

        // ROS messages
        std_msgs::Float64 motor_msg_;
        std_msgs::Float64 servo_msg_;

        // ROS callbacks
        void ackermannCmdCallback(const ackermann_msgs::AckermannDriveStamped::ConstPtr& cmd);

        // timer callback to publish
        void pubMotorTimerCb(const ros::TimerEvent& event);
        void pubServoTimerCb(const ros::TimerEvent& event);
    };

} // namespace vesc_interface

#endif // CMD_PUBLISHER_H_
