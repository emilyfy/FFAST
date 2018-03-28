#ifndef _AVOID_OBS_ILQR_H_
#define _AVOID_OBS_ILQR_H_

#define HORIZON   10
#define MAX_ITER 5000

//// REMEMBER TO CHANGE THIS IF THE iLQG_func.c FILE IS CHANGED!!!!!!! ////
#define P_XDES_IDX 28
#define P_OBS_IDX   3

#include <math.h>
#include <boost/random.hpp>

#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <obstacle_detector/Obstacles.h>
#include <obstacle_detector/CircleObstacle.h>
#include <evasive_maneuver/IlqrInput.h>
#include <evasive_maneuver/IlqrOutput.h>

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
    double x0_[10], u0_[HORIZON*2], xDes_[6], Obs_[2];
    tOptSet* Op_;
    geometry_msgs::Pose2D state_;

    // ROS callbacks
    void ilqrCb(const evasive_maneuver::IlqrInput::ConstPtr& msg);
    void obsCb(const obstacle_detector::Obstacles::ConstPtr& msg);

    // Function
    int assignParams(ros::NodeHandle nh, ros::NodeHandle pnh);
};

#endif