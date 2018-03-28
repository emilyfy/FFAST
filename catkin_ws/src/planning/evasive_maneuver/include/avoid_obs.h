#ifndef _AVOID_OBS_H_
#define _AVOID_OBS_H_

#include <math.h>

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Odometry.h>
#include <obstacle_detector/Obstacles.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/PoseStamped.h>
#include <obstacle_detector/CircleObstacle.h>
#include <evasive_maneuver/IlqrInput.h>
#include <evasive_maneuver/IlqrOutput.h>
#include <nav_msgs/Path.h>

class AvoidObs
{
  public:
    AvoidObs(ros::NodeHandle nh, ros::NodeHandle pnh);
    void spin(void);

  private:
    // ROS Handles
    ros::Subscriber state_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber ilqr_sub_;
    ros::Publisher goal_pub_;
    ros::Publisher cmd_pub_;
    ros::Publisher ilqr_pub_;
    ros::Publisher path_pub_;
    
    // ROS messages
    evasive_maneuver::IlqrOutput plan_;
    ackermann_msgs::AckermannDriveStamped cmd_;
    evasive_maneuver::IlqrInput ilqr_input_;
    nav_msgs::Path path_;

    evasive_maneuver::State state_;
    geometry_msgs::Pose2D goal_;
    bool goal_set_;
    double dist_to_goal_;
    int cmd_on_plan_;

    ros::Rate cmd_rate_;

    double clamp_(double val, double lower_limit, double upper_limit);
    void stateCb(const nav_msgs::Odometry::ConstPtr& msg);
    void goalCb(const geometry_msgs::PoseStamped::ConstPtr& msg);
    void ilqrCb(const evasive_maneuver::IlqrOutput &msg);
};

#endif