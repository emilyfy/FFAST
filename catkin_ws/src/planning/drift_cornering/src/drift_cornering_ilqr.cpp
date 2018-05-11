#include "drift_cornering_ilqr.h"

#include <ilqr_msgs/IlqrOutput.h>

iLQR::iLQR(ros::NodeHandle nh, ros::NodeHandle pnh) :
    equilibrium_reached_(false), equilibrium_(5), N_(HORIZON_STATIONARY+1)
{
    _setOpStationary(nh, pnh);
    _setOpSteadyState(nh, pnh);

    // initialize vars
    ros::Duration(2).sleep();    // wait for eq param to be set
    if (!nh.getParam("equilibrium", equilibrium_)) {
        ROS_ERROR("Failed to get equilibrium solution parameter.");
        return;
    }
    vel_dist.param(std::normal_distribution<double>::param_type(equilibrium_[0]*0.8,0.2));
    steer_dist.param(std::normal_distribution<double>::param_type(equilibrium_[4]*0.07,0.1));

    // setup ROS subscribers and publishers
    ilqr_sub_ = nh.subscribe("ilqr_input", 1, &iLQR::ilqrCb, this);
    ilqr_pub_ = nh.advertise<ilqr_msgs::IlqrOutput>("ilqr_output", 1);
    timer_sub_ = nh.subscribe("timer", 1, &iLQR::timerCb, this);
}

void iLQR::timerCb(const ilqr_msgs::BoolStamped::ConstPtr& msg)
{
    equilibrium_reached_ = msg->data; 

    if (!equilibrium_reached_) {
        N_ = HORIZON_STATIONARY + 1;
    }
    else {
        N_ = HORIZON_STEADYSTATE + 1;
    }
}

void iLQR::ilqrCb(const ilqr_msgs::IlqrInput::ConstPtr& msg)
{
    if (!equilibrium_reached_) _ilqrStationary(msg);
    else if (equilibrium_reached_) _ilqrSteadyState(msg);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "drift_cornering_ilqr");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    iLQR ilqr(nh, pnh);

    ros::spin();

    return 0;
}