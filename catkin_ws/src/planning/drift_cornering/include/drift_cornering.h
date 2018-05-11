#ifndef _DRIFT_CORNERING_H_
#define _DRIFT_CORNERING_H_

#include <math.h>
#include <vector>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ilqr_msgs/IlqrInput.h>
#include <ilqr_msgs/IlqrOutput.h>
#include <ilqr_msgs/BoolStamped.h>

#include "equilibria.h"

class DriftCornering
{
  public:
    DriftCornering(ros::NodeHandle nh, ros::NodeHandle pnh);
    void spin(void);

  private:
    // ROS Handles
    ros::Subscriber state_sub_;
    ros::Subscriber ilqr_sub_;
    ros::Publisher cmd_pub_;
    ros::Publisher timer_pub_;
    ros::Publisher ilqr_pub_;
    ros::Publisher path_pub_;
    
    // ROS messages
    ilqr_msgs::IlqrOutput plan_;
    ackermann_msgs::AckermannDriveStamped cmd_;
    ilqr_msgs::IlqrInput ilqr_input_;
    nav_msgs::Path path_;

    // Variables
    ilqr_msgs::State state_;
    bool state_rcv_;
    
    int cmd_on_plan_;
    ros::Rate cmd_rate_;

    int eq_steering_angle_;
    bool eq_countersteering_;
    std::vector<double> equilibrium_;

    bool equilibrium_reached_;
    ros::Time equilibrium_start_time_;
    ros::Duration equilibrium_time_;

    // Functions
    double clamp_(double val, double lower_limit, double upper_limit);
    void stateCb(const nav_msgs::Odometry::ConstPtr& msg);
    void ilqrCb(const ilqr_msgs::IlqrOutput &msg);
    void setEquilibrium_(int steering_angle, bool countersteering);
};

#endif