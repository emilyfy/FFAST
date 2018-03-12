// Receives wheel speed data from vesc and yaw data from IMU
// Publishes odom and tf if requested

// Param predict_slip when set predicts wheel slip based on accel/brake
// assumes linear relation between accel and slip ratio
// requires params slip_gain and slip_offset

// x covariance increase with speed
// y covariance increase with ay
// yaw covariance increase with yaw rate

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TransformStamped.h>
#include <vesc_msgs/VescStateStamped.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_broadcaster.h>
#include <cmath>
#include <tf/transform_datatypes.h>
#include <boost/shared_ptr.hpp>

class FusedOdomPublisher
{
    public:
        FusedOdomPublisher(ros::NodeHandle nh, ros::NodeHandle pnh);

    private:
        double x_, y_, yaw_;
        double slip_tix_gain_;
        ros::Time last_imu_time_;

        // ROS params
        double speed_gain_, speed_offset_;
        double slip_gain_speed_, slip_offset_speed_;
        double slip_gain_accel_, slip_offset_accel_;
        double slip_gain_brake_, slip_offset_brake_;
        double geometric_tix_gain_;
        bool publish_tf_;
        bool predict_slip_;

        // ROS messages
        sensor_msgs::Imu::ConstPtr last_imu_;
        vesc_msgs::VescStateStamped::ConstPtr last_state_;
        nav_msgs::Odometry odom_;
        geometry_msgs::TransformStamped tf_;

        // ROS services
        ros::Subscriber vesc_state_sub_;
        ros::Subscriber imu_sub_;
        ros::Publisher odom_pub_;
        boost::shared_ptr<tf::TransformBroadcaster> tf_pub_;

        // ROS callbacks
        void vescStateCallback(const vesc_msgs::VescStateStamped::ConstPtr& state);
        void imuCallback(const sensor_msgs::Imu::ConstPtr& servo);
};


template <typename T>
inline bool getRequiredParam(const ros::NodeHandle& n, std::string name, T& value);

FusedOdomPublisher::FusedOdomPublisher(ros::NodeHandle nh, ros::NodeHandle pnh) :
    publish_tf_(true), predict_slip_(false), x_(0.0), y_(0.0), yaw_(0.0)
{
    if (!getRequiredParam(nh, "vesc_interface/speed_gain", speed_gain_))
        return;
    if (!getRequiredParam(nh, "vesc_interface/speed_offset", speed_offset_))
        return;
    if (!getRequiredParam(nh, "vesc_interface/tix_gain", geometric_tix_gain_))
        return;
    pnh.param("publish_tf", publish_tf_, publish_tf_);
    pnh.param("predict_slip", predict_slip_, predict_slip_);
    if (predict_slip_)
    {
        if (!getRequiredParam(pnh, "slip_gain_accel", slip_gain_accel_))
            return;
        if (!getRequiredParam(pnh, "slip_offset_accel", slip_offset_accel_))
            return;
        if (!getRequiredParam(pnh, "slip_gain_brake", slip_gain_brake_))
            return;
        if (!getRequiredParam(pnh, "slip_offset_brake", slip_offset_brake_))
            return;
        if (!getRequiredParam(pnh, "slip_gain_speed", slip_gain_speed_))
            return;
        if (!getRequiredParam(pnh, "slip_offset_speed", slip_offset_speed_))
            return;
    }
    else
        slip_tix_gain_ = geometric_tix_gain_;

    // create odom publisher
    odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom_fused", 10);

    // create tf broadcaster
    if (publish_tf_) {
        tf_pub_.reset(new tf::TransformBroadcaster);
    }

    // subscribe to vesc state and servo command
    vesc_state_sub_ = nh.subscribe("sensors/core", 10, &FusedOdomPublisher::vescStateCallback, this);
    imu_sub_ = nh.subscribe("imu", 10, &FusedOdomPublisher::imuCallback, this);

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

    last_imu_time_ = ros::Time::now();
}

void FusedOdomPublisher::vescStateCallback(const vesc_msgs::VescStateStamped::ConstPtr& state)
{

    // use current state as last state if this is our first time here
    if (!last_state_)
        last_state_ = state;

    // wait for first imu msg
    if (!last_imu_)
        return;

    // check for imu timeout
    if ( (ros::Time::now()-last_imu_time_).toSec() > 1.0 ) {
        ROS_ERROR("IMU sensor timed out. Not publishing fused_odom.");
        return;
    }

    // get vesc state
    double wheel_speed = ( state->state.speed - speed_offset_ ) / speed_gain_;
    double current_disp = state->state.displacement;
    static double last_disp = current_disp;
    
    // calculate accel dependent tix gain
    double slip_ratio;
    if (predict_slip_) {
        double accel = last_imu_->linear_acceleration.x * -1.0;  // our imu x axis is -ve of robot x axis
        accel *= (wheel_speed>0 ? 1 : -1);                       // account for when car moving backwards
        if (accel > 0)                                           // accelerating
            slip_ratio = slip_gain_accel_ * fmax(0,accel-slip_offset_accel_) + slip_gain_speed_ * fmax(0,wheel_speed-slip_offset_speed_);
        else                                                     // braking
            slip_ratio = slip_gain_brake_ * fmax(0,-1.0*accel-slip_offset_brake_) + slip_gain_speed_ * fmax(0,wheel_speed-slip_offset_speed_);
        slip_tix_gain_ = geometric_tix_gain_ * (1 + slip_ratio);
    }
    
    // calc elapsed time
    ros::Duration dt = state->header.stamp - last_state_->header.stamp;

    // propigate odometry
    double ds = (current_disp - last_disp) / slip_tix_gain_;
    yaw_ = tf::getYaw(last_imu_->orientation);
    x_ += ds * cos(yaw_);
    y_ += ds * sin(yaw_);

    // calculate covariance
    odom_.pose.covariance[0] = 0.1 + 0.001 * fabs(wheel_speed);
    odom_.twist.covariance[0] = 0.1 + 0.001 * fabs(wheel_speed);
    odom_.pose.covariance[6*1+1] = 0.5 + 0.5 * fmax(0,fabs(last_imu_->linear_acceleration.y)-0.2);
    odom_.twist.covariance[6*1+1] = 0.5 + 0.5 * fmax(0,fabs(last_imu_->linear_acceleration.y)-0.2);
    odom_.pose.covariance[6*5+5] = 0.1 * 0.05 * fmax(0,fabs(last_imu_->angular_velocity.z)-0.1);
    odom_.twist.covariance[6*5+5] = 0.1 * 0.1 * fmax(0,fabs(last_imu_->angular_velocity.z)-0.1);

    // save state for next time
    last_state_ = state;
    last_disp = current_disp;
    
    // publish odometry message
    odom_.header.stamp = state->header.stamp;
    odom_.pose.pose.position.x = x_;
    odom_.pose.pose.position.y = y_;
    odom_.pose.pose.orientation.z = sin(yaw_/2.0);
    odom_.pose.pose.orientation.w = cos(yaw_/2.0);

    // Velocity (in the coordinate frame given by the child_frame_id) accounting for wheel slip
    if (predict_slip_) {
        odom_.twist.twist.linear.x = wheel_speed / (1 + slip_ratio);
    } else odom_.twist.twist.linear.x = wheel_speed;
    odom_.twist.twist.angular.z = last_imu_->angular_velocity.z;

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

void FusedOdomPublisher::imuCallback(const sensor_msgs::Imu::ConstPtr& imu)
{
    last_imu_time_ = imu->header.stamp;
    last_imu_ = imu;
}

template <typename T>
inline bool getRequiredParam(const ros::NodeHandle& n, std::string name, T& value)
{
  if (n.getParam(name, value))
    return true;

  ROS_FATAL("OdomPublisher: Parameter %s is required.", name.c_str());
  return false;
}


int main(int argc, char** argv)
{
  ros::init(argc, argv, "fused_odom_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  FusedOdomPublisher fused_odom_publisher(nh, pnh);

  ros::spin();

  return 0;
}