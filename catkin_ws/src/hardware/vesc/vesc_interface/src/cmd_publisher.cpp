#include "vesc_interface/cmd_publisher.h"

#include <cmath>
#include <sstream>

#include <std_msgs/Float64.h>

namespace vesc_interface
{
    
    template <typename T>
    inline bool getRequiredParam(const ros::NodeHandle& n, std::string name, T& value);

    CmdPublisher::CmdPublisher(ros::NodeHandle nh, ros::NodeHandle pnh) :
        max_motor_accel_(0.0), motor_smoother_rate_(0.0),
        max_servo_speed_(0.0), servo_smoother_rate_(0.0),
        max_speed_chg_(0.0), max_pos_chg_(0.0)
    {
        // get conversion and smoothing parameters
        if (!getRequiredParam(pnh, "speed_gain", speed_gain_))
            return;
        if (!getRequiredParam(pnh, "speed_offset", speed_offset_))
            return;
        if (!getRequiredParam(pnh, "steering_gain", steering_gain_))
            return;
        if (!getRequiredParam(pnh, "steering_offset", steering_offset_))
            return;
        
        if (!pnh.getParam("max_motor_accel", max_motor_accel_)) {
            limit_motor_ = false;
        } else limit_motor_ = true;
        if (!pnh.getParam("motor_smoother_rate", motor_smoother_rate_)) {
            smooth_motor_ = false;
        } else smooth_motor_ = true;
        if (!pnh.getParam("max_servo_speed", max_servo_speed_)) {
            limit_servo_ = false;
        } else limit_servo_ = true;
        if (!pnh.getParam("servo_smoother_rate", servo_smoother_rate_)) {
            smooth_servo_ = false;
        } else smooth_servo_ = true;

        // initialize variables
        last_speed_ = 0;
        target_speed_ = last_speed_;
        last_pos_ = steering_offset_;
        target_pos_ = last_pos_;
        
        if (limit_motor_ && smooth_motor_)
            max_speed_chg_ = abs(speed_gain_ * max_motor_accel_ / motor_smoother_rate_);
        if (limit_servo_ && smooth_servo_)
            max_pos_chg_ = abs(steering_gain_ * max_servo_speed_ / servo_smoother_rate_);

        // create publishers to vesc speed and servo commands
        motor_pub_ = nh.advertise<std_msgs::Float64>("commands/motor/speed", 10);
        servo_pub_ = nh.advertise<std_msgs::Float64>("commands/servo/position", 10);

        // subscribe to ackermann topic
        ackermann_sub_ = nh.subscribe("commands/ackermann", 10, &CmdPublisher::ackermannCmdCallback, this);
        
        // create timers to publish command based on smoother rate
        if (smooth_motor_)
            motor_pub_timer_ = nh.createTimer(ros::Duration(1.0/motor_smoother_rate_), &CmdPublisher::pubMotorTimerCb, this);
        if (smooth_servo_)
            servo_pub_timer_ = nh.createTimer(ros::Duration(1.0/servo_smoother_rate_), &CmdPublisher::pubServoTimerCb, this);

    }

    typedef ackermann_msgs::AckermannDriveStamped::ConstPtr AckermannMsgPtr;
    void CmdPublisher::ackermannCmdCallback(const AckermannMsgPtr& cmd)
    {
        target_speed_ = speed_gain_ * cmd->drive.speed + speed_offset_;
        target_pos_ = steering_gain_ * cmd->drive.steering_angle + steering_offset_;
        
        if (!smooth_motor_)
        {
            double chg_speed = target_speed_ - last_speed_;
            if (limit_motor_) {
                ros::Time curr_time = ros::Time::now();
                ros::Duration dt = curr_time - last_pub_motor_time_;
                max_speed_chg_ = fabs(speed_gain_ * max_motor_accel_ * dt.toSec());
                chg_speed = std::max(std::min(chg_speed, max_speed_chg_), -max_speed_chg_);
            }
            last_speed_ += chg_speed;
            
            motor_msg_.data = last_speed_;

            if (ros::ok()) {
                motor_pub_.publish(motor_msg_);
            }
        }

        if (!smooth_servo_)
        {
            double chg_pos = target_pos_ - last_pos_;
            if (limit_servo_) {
                ros::Time curr_time = ros::Time::now();
                ros::Duration dt = curr_time - last_pub_servo_time_;
                max_pos_chg_ = fabs(steering_gain_ * max_servo_speed_ * dt.toSec());
                chg_pos = std::max(std::min(chg_pos, max_pos_chg_), -max_pos_chg_);
            }
            last_pos_ += chg_pos;
            
            servo_msg_.data = last_pos_;
            if (ros::ok()) {
                servo_pub_.publish(servo_msg_);
            }
        }

    }

    void CmdPublisher::pubMotorTimerCb(const ros::TimerEvent& event)
    {
        double chg_speed = target_speed_ - last_speed_;
        if (limit_motor_) {
            chg_speed = std::max(std::min(chg_speed, max_speed_chg_), -max_speed_chg_);
        }
        last_speed_ += chg_speed;
        motor_msg_.data = last_speed_;
        motor_pub_.publish(motor_msg_);
    }

    void CmdPublisher::pubServoTimerCb(const ros::TimerEvent& event)
    {
        double chg_pos = target_pos_ - last_pos_;
        if (limit_servo_) {
            double chg_pos = std::max(std::min(chg_pos, max_pos_chg_), -max_pos_chg_);
        }
        last_pos_ += chg_pos;
        servo_msg_.data = last_pos_;
        servo_pub_.publish(servo_msg_);
    }

    template <typename T>
    inline bool getRequiredParam(const ros::NodeHandle& n, std::string name, T& value)
    {
        if (n.getParam(name, value))
            return true;

        ROS_FATAL("CmdPublisher: Parameter %s is required.", name.c_str());
        return false;
    }

} // namespace vesc_interface
