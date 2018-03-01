#include "vesc_interface/odom_publisher.h"

#include <cmath>

#include <nav_msgs/Odometry.h>

namespace vesc_interface
{

template <typename T>
inline bool getRequiredParam(const ros::NodeHandle& n, std::string name, T& value);

OdomPublisher::OdomPublisher(ros::NodeHandle nh, ros::NodeHandle pnh) :
    publish_tf_(false), x_(0.0), y_(0.0), yaw_(0.0)
{
    if (!getRequiredParam(nh, "vesc_interface/speed_gain", speed_gain_))
        return;
    if (!getRequiredParam(nh, "vesc_interface/speed_offset", speed_offset_))
        return;
    if (!getRequiredParam(nh, "vesc_interface/steering_gain", steering_gain_))
        return;
    if (!getRequiredParam(nh, "vesc_interface/steering_offset", steering_offset_))
        return;
    if (!getRequiredParam(nh, "vehicle/wheelbase", wheelbase_))
        return;
    if (!getRequiredParam(nh, "vehicle/wheelbase_r", wheelbase_r_))
        return;
    if (!getRequiredParam(nh, "vesc_interface/tix_gain", tix_gain_))
        return;
    
    // TODO - think about covariance
    // should it be higher? should it increase with accel?
    // maybe tix_gain should vary with accel?
    // should we subscribe to imu? and also trust imu heading more than steering angle for yaw?

    pnh.param("publish_tf", publish_tf_, publish_tf_);
    if (pnh.getParam("odom_covariance", odom_covariance_)) {
        for(int i=0;i<6;i++) {
                odom_.pose.covariance[6*i+i] = odom_covariance_;
                odom_.twist.covariance[6*i+i] = odom_covariance_;
        }
    }
    // create odom publisher
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 10);

    // create tf broadcaster
    if (publish_tf_) {
        tf_pub_.reset(new tf::TransformBroadcaster);
    }

    // subscribe to vesc state and. optionally, servo command
    vesc_state_sub_ = nh.subscribe("sensors/core", 10, &OdomPublisher::vescStateCallback, this);
    servo_sub_ = nh.subscribe("sensors/servo_position_command", 10, &OdomPublisher::servoCmdCallback, this);

    // set static values in odom_
    odom_.header.frame_id = "odom";
    odom_.child_frame_id = "base_link";
    odom_.pose.pose.position.z = 0.0;
    odom_.pose.pose.orientation.x = 0.0;
    odom_.pose.pose.orientation.y = 0.0;
    odom_.twist.twist.linear.y = 0.0;

    // set static values in tf_
    if (publish_tf_)
    {
        tf_.header.frame_id = "odom";
        tf_.child_frame_id = "base_link";
        tf_.transform.translation.z = 0.0;
    }
}

void OdomPublisher::vescStateCallback(const vesc_msgs::VescStateStamped::ConstPtr& state)
{
    
    // convert to engineering units
    double current_speed = ( state->state.speed - speed_offset_ ) / speed_gain_;
    double current_steering_angle;    // check that we have a last servo command if we are depending on it for angular velocity
    if (!last_servo_cmd_)
        current_steering_angle = 0.0;
    else
        current_steering_angle = ( last_servo_cmd_->data - steering_offset_ ) / steering_gain_;
    
    double current_beta = atan( wheelbase_r_ * cos(current_beta) * tan(current_steering_angle) / wheelbase_ );
    double current_angular_velocity = current_speed * tan(current_steering_angle) / wheelbase_;
    
    double current_disp = state->state.displacement;
    static double last_disp = current_disp;
    double chg_disp = (current_disp - last_disp) / tix_gain_;
    last_disp = current_disp;
    
    // use current state as last state if this is our first time here
    if (!last_state_)
        last_state_ = state;

    // calc elapsed time
    ros::Duration dt = state->header.stamp - last_state_->header.stamp;

    // propigate odometry
    x_ += chg_disp * cos(yaw_ + current_beta);
    y_ += chg_disp * sin(yaw_ + current_beta);
    yaw_ += current_angular_velocity * dt.toSec();

    // save state for next time
    last_state_ = state;

    // publish odometry message
    odom_.header.stamp = state->header.stamp;
    odom_.pose.pose.position.x = x_;
    odom_.pose.pose.position.y = y_;
    odom_.pose.pose.orientation.z = sin(yaw_/2.0);
    odom_.pose.pose.orientation.w = cos(yaw_/2.0);
    //odom_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw_);

    // Velocity ("in the coordinate frame given by the child_frame_id")
    odom_.twist.twist.linear.x = current_speed;
    odom_.twist.twist.angular.z = current_angular_velocity;

    if (publish_tf_) {
        tf_.header.stamp = ros::Time::now();
        tf_.transform.translation.x = x_;
        tf_.transform.translation.y = y_;
        tf_.transform.rotation = odom_.pose.pose.orientation;
        if (ros::ok()) {
            tf_pub_->sendTransform(tf_);
        }
    }

    if (ros::ok()) {
        odom_pub_.publish(odom_);
    }
}

void OdomPublisher::servoCmdCallback(const std_msgs::Float64::ConstPtr& servo)
{
    last_servo_cmd_ = servo;
}

template <typename T>
inline bool getRequiredParam(const ros::NodeHandle& n, std::string name, T& value)
{
  if (n.getParam(name, value))
    return true;

  ROS_FATAL("OdomPublisher: Parameter %s is required.", name.c_str());
  return false;
}

} // namespace vesc_interface
