#ifndef _AVOID_OBS_ILQR_H_
#define _AVOID_OBS_ILQR_H_

#define HORIZON   10
#define MAX_ITER  50

//// REMEMBER TO CHANGE THIS IF THE iLQG_func.c FILE IS CHANGED!!!!!!! ////
#define P_OBS_VEL_IDX 23
#define P_GOAL_IDX    13
#define P_CF_IDX      8

#define CHANGE_CF_DIST 0.1

#include <math.h>
#include <boost/random.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <obstacle_detector/Obstacles.h>
#include <ilqr_msgs/IlqrInput.h>
#include <ilqr_msgs/IlqrOutput.h>

extern "C" {
  #include "iLQG.h"
  #include "iLQG_problem.h"
  #include "matMult.h"
}

class iLQR
{
  public:
    iLQR(ros::NodeHandle nh, ros::NodeHandle pnh);

  private:
    // ROS Handles
    ros::Subscriber ilqr_sub_;
    ros::Subscriber obs_sub_;
    ros::Publisher ilqr_pub_;
    
    // random number generator
    std::mt19937 vel_gen, steer_gen;
    std::normal_distribution<double> vel_dist, steer_dist;

    // variables
    int N_;
    tOptSet* Op_;
    double x0_[10], u0_[HORIZON*2], obs_vel_[2];
    double cf_bef_goal_[6], cf_aft_goal_[6];

    // ROS callbacks
    void ilqrCb(const ilqr_msgs::IlqrInput::ConstPtr& msg);
    void obsCb(const obstacle_detector::Obstacles::ConstPtr& msg);

    // Function
    int assignParams(ros::NodeHandle nh, ros::NodeHandle pnh);
};

#endif