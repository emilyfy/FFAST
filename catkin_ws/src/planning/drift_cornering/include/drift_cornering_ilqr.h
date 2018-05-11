#ifndef _DRIFT_CORNERING_ILQR_H_
#define _DRIFT_CORNERING_ILQR_H_

#define HORIZON_STATIONARY   50
#define HORIZON_STEADYSTATE   1
#define MAX_ITER            500

//// REMEMBER TO CHANGE THIS IF THE iLQG_func.c FILE IS CHANGED!!!!!!! ////
#define P_GOAL_IDX_STATIONARY   10
#define P_GOAL_IDX_STEADYSTATE   9

#include <math.h>
#include <vector>
#include <boost/random.hpp>

#include <ros/ros.h>
#include <ilqr_msgs/IlqrInput.h>
#include <ilqr_msgs/IlqrOutput.h>
#include <ilqr_msgs/BoolStamped.h>

class iLQR
{
  public:
    iLQR(ros::NodeHandle nh, ros::NodeHandle pnh);

  private:
    // ROS Handles
    ros::Subscriber ilqr_sub_;
    ros::Publisher ilqr_pub_;
    ros::Subscriber timer_sub_;
    
    // random number generator
    std::mt19937 vel_gen, steer_gen;
    std::normal_distribution<double> vel_dist, steer_dist;

    // variables
    int N_;
    std::vector<double> equilibrium_;
    bool equilibrium_reached_;
    double x0_[8];
    double u0_stationary_[HORIZON_STATIONARY*2], u0_steadystate_[HORIZON_STEADYSTATE*2];

    // ROS callbacks
    void ilqrCb(const ilqr_msgs::IlqrInput::ConstPtr& msg);
    void timerCb(const ilqr_msgs::BoolStamped::ConstPtr& msg);

    // Functions
    void _setOpStationary(ros::NodeHandle nh, ros::NodeHandle pnh);
    void _setOpSteadyState(ros::NodeHandle nh, ros::NodeHandle pnh);
    int _assignParamsStationary(ros::NodeHandle nh, ros::NodeHandle pnh);
    int _assignParamsSteadyState(ros::NodeHandle nh, ros::NodeHandle pnh);
    void _ilqrStationary(const ilqr_msgs::IlqrInput::ConstPtr& msg);
    void _ilqrSteadyState(const ilqr_msgs::IlqrInput::ConstPtr& msg);
};

#endif