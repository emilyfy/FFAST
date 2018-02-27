#ifndef ODOM_PUBLISHER_H_
#define ODOM_PUBLISHER_H_

#include <ros/ros.h>
#include <vesc_msgs/VescStateStamped.h>
#include <std_msgs/Float64.h>
#include <boost/shared_ptr.hpp>
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>

namespace vesc_interface
{
    class OdomPublisher
    {
        public:

            OdomPublisher(ros::NodeHandle nh, ros::NodeHandle pnh);

        private:
            double speed_gain_, speed_offset_;
            double steering_gain_, steering_offset_;
            double tix_gain_;
            double odom_covariance_;
            double wheelbase_, wheelbase_r_;
            bool publish_tf_;

            // odometry state
            double x_, y_, yaw_;
            std_msgs::Float64::ConstPtr last_servo_cmd_; ///< Last servo position commanded value
            vesc_msgs::VescStateStamped::ConstPtr last_state_; ///< Last received state message

            // ROS services
            ros::Publisher odom_pub_;
            ros::Subscriber vesc_state_sub_;
            ros::Subscriber servo_sub_;
            boost::shared_ptr<tf::TransformBroadcaster> tf_pub_;

            nav_msgs::Odometry odom_;
            geometry_msgs::TransformStamped tf_;

            // ROS callbacks
            void vescStateCallback(const vesc_msgs::VescStateStamped::ConstPtr& state);
            void servoCmdCallback(const std_msgs::Float64::ConstPtr& servo);
    };

} // namespace vesc_interface

#endif // ODOM_PUBLISHER_H_
