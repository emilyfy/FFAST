#include "drift_cornering.h"

#define CMD_RATE               50
#define MIN_EQUILIBRIUM_TIME    5
#define V_X_TOLERANCE     0.5
#define BETA_TOLERANCE    0.2
#define YAWRATE_TOLERANCE 1.0

#define VEL_KP   0.001
#define VEL_KI   0
#define VEL_KD   0
#define YAW_KP   0.001
#define YAW_KI   0
#define YAW_KD   0
#define VEL_ERR_INT_LIMIT 100
#define YAW_ERR_INT_LIMIT 100

DriftCornering::DriftCornering(ros::NodeHandle nh, ros::NodeHandle pnh) : 
    cmd_on_plan_(0), state_rcv_(false), cmd_rate_(CMD_RATE), equilibrium_(5), equilibrium_reached_(false), equilibrium_time_(0)
{
    // setup publishers and subscribers
    state_sub_  = nh.subscribe("ekf_localization/odom", 10, &DriftCornering::stateCb, this);
    ilqr_sub_ = nh.subscribe("ilqr_output", 1, &DriftCornering::ilqrCb, this);
    cmd_pub_ = nh.advertise<ackermann_msgs::AckermannDriveStamped>("commands/cmd_vel", 5);
    timer_pub_ = nh.advertise<ilqr_msgs::BoolStamped>("timer", 1);
    ilqr_pub_ = nh.advertise<ilqr_msgs::IlqrInput>("ilqr_input", 1);
    path_pub_ = nh.advertise<nav_msgs::Path>("ilqr_path", 1);

    // set equilibriium solution
    if (!pnh.getParam("eq_steering_angle", eq_steering_angle_)) eq_steering_angle_ = 10;
    if (!pnh.getParam("eq_countersteering", eq_countersteering_)) eq_countersteering_ = true;
    // setEquilibrium_(eq_steering_angle_, eq_countersteering_);
    equilibrium_[0] = 3; equilibrium_[1] = 0; equilibrium_[2] = 1.2897; equilibrium_[3] = 0.7868; equilibrium_[4] = -4.2959;
    nh.setParam("equilibrium", equilibrium_);

    // wait for iLQR node and localization to start
    ros::Duration(5).sleep();
    while(ros::ok() && !state_rcv_) ros::spinOnce();

    // publish the first iLQR input
    ilqr_input_.header.stamp = ros::Time::now();
    ilqr_input_.state = state_;
    ilqr_input_.remainingcommands.clear();
    ilqr_pub_.publish(ilqr_input_);

    ilqr_msgs::BoolStamped timer_start;
    timer_start.header.stamp = ilqr_input_.header.stamp;
    timer_start.data = false;
    timer_pub_.publish(timer_start);

    ROS_INFO("Starting drift cornering from stationary.");
}

void DriftCornering::setEquilibrium_(int steering_angle, bool countersteering)
{
    int idx = steering_angle/5*2;
    if (!countersteering) idx += 1;
    for (int i=0; i<5; i++) equilibrium_[i] = equilibria_[idx][i];
}

void DriftCornering::stateCb(const nav_msgs::Odometry::ConstPtr& msg)
{
    if (!state_rcv_) state_rcv_ = true;
    
    state_.pose.x = msg->pose.pose.position.x;
    state_.pose.y = msg->pose.pose.position.y;
    state_.pose.theta = tf::getYaw(msg->pose.pose.orientation);
    state_.twist.x = msg->twist.twist.linear.x;
    state_.twist.y = msg->twist.twist.linear.y;
    state_.twist.theta = msg->twist.twist.angular.z;

    // check for equilibrium
    if ( ( fabs(state_.twist.x-equilibrium_[2]) < V_X_TOLERANCE && 
           fabs(atan2(state_.twist.y,state_.twist.x)-atan2(equilibrium_[3],equilibrium_[2])) < BETA_TOLERANCE &&
           fabs(state_.twist.theta-equilibrium_[4]) ) < YAWRATE_TOLERANCE ) {
        if (!equilibrium_reached_) {
            equilibrium_reached_ = true;
            ROS_INFO("Reached equilibrium.");
        }
    } else {
        if (equilibrium_reached_) {
            equilibrium_reached_ = false;
            ROS_INFO("Gone out of equilibrium.");
        }
    }
}

void DriftCornering::ilqrCb(const ilqr_msgs::IlqrOutput &msg)
{
    ilqr_input_.header.stamp = ros::Time::now();
    ilqr_input_.state = state_;
    ilqr_input_.previouscommand = cmd_.drive;
    ilqr_input_.remainingcommands.clear();
    for (int i=cmd_on_plan_; i<plan_.commands.size(); i++)
        ilqr_input_.remainingcommands.push_back(plan_.commands[i]);
    ilqr_pub_.publish(ilqr_input_);

    plan_ = msg;
    cmd_on_plan_ = 0;

    path_.header.stamp = ros::Time::now();
    path_.header.frame_id = "map";
    path_.poses.resize(plan_.states.size());
    for (int i=0; i<plan_.states.size(); i++) {
        path_.poses[i].header.stamp = path_.header.stamp + ros::Duration(i*0.02);
        path_.poses[i].header.frame_id = "map";
        path_.poses[i].pose.position.x = plan_.states[i].pose.x;
        path_.poses[i].pose.position.y = plan_.states[i].pose.y;
        path_.poses[i].pose.orientation = tf::createQuaternionMsgFromYaw(plan_.states[i].pose.theta);
    }
    path_pub_.publish(path_);
}

void DriftCornering::spin(void)
{
    while (ros::ok() && equilibrium_time_.toSec() < MIN_EQUILIBRIUM_TIME)
    {
        while (ros::ok() && !equilibrium_reached_)
        {
            cmd_.header.stamp = ros::Time::now();

            if (cmd_on_plan_ >= plan_.commands.size()) {
                ROS_ERROR("Planner reached end of iLQR command sequence.");
                cmd_.drive.speed = equilibrium_[0]*0.8;
                cmd_.drive.steering_angle = equilibrium_[4]*0.07;
            }
            else {
                cmd_.drive = plan_.commands[cmd_on_plan_];
                cmd_on_plan_++;

                // PID on v_x and yaw
                double vel_err = plan_.states[cmd_on_plan_].twist.x - state_.twist.x;
                static double vel_err_prev = vel_err;
                static double vel_err_int = 0;
                double yaw_err = plan_.states[cmd_on_plan_].pose.theta - state_.pose.theta;
                static double yaw_err_prev = yaw_err;
                static double yaw_err_int = 0;

                cmd_.drive.speed += VEL_KP*vel_err + -VEL_KI*vel_err_int + VEL_KD*(vel_err-vel_err_prev);
                cmd_.drive.steering_angle += YAW_KP*yaw_err + -YAW_KI*yaw_err_int + YAW_KD*(yaw_err-yaw_err_prev);

                vel_err_prev = vel_err;
                vel_err_int += vel_err/(double)CMD_RATE;
                vel_err_int = clamp_(vel_err, -VEL_ERR_INT_LIMIT, VEL_ERR_INT_LIMIT);
                yaw_err_prev = yaw_err;
                yaw_err_int += yaw_err/(double)CMD_RATE;
                yaw_err_int = clamp_(yaw_err, -YAW_ERR_INT_LIMIT, YAW_ERR_INT_LIMIT);
            }

            cmd_pub_.publish(cmd_);
            ros::spinOnce();
            cmd_rate_.sleep();
        }

        if (equilibrium_reached_)
        {
            ilqr_msgs::BoolStamped timer_reach;
            timer_reach.header.stamp = equilibrium_start_time_;
            timer_reach.data = true;
            timer_pub_.publish(timer_reach);
        }
        
        while (ros::ok() && equilibrium_reached_ && equilibrium_time_.toSec() < MIN_EQUILIBRIUM_TIME)
        {
            ros::Time curr_time = ros::Time::now();
            equilibrium_time_ = curr_time - equilibrium_start_time_;
            cmd_.header.stamp = curr_time;

            if (cmd_on_plan_ >= plan_.commands.size()) {
                ROS_ERROR("Planner reached end of iLQR command sequence.");
                cmd_.drive.speed = equilibrium_[0];
                cmd_.drive.steering_angle = equilibrium_[1];
            }
            else {
                cmd_.drive = plan_.commands[cmd_on_plan_];
                cmd_on_plan_++;

                // PID on v_x and yaw
                double vel_err = plan_.states[cmd_on_plan_].twist.x - state_.twist.x;
                static double vel_err_prev = vel_err;
                static double vel_err_int = 0;
                double yaw_err = plan_.states[cmd_on_plan_].pose.theta - state_.pose.theta;
                static double yaw_err_prev = yaw_err;
                static double yaw_err_int = 0;

                cmd_.drive.speed += VEL_KP*vel_err + -VEL_KI*vel_err_int + VEL_KD*(vel_err-vel_err_prev);
                cmd_.drive.steering_angle += YAW_KP*yaw_err + -YAW_KI*yaw_err_int + YAW_KD*(yaw_err-yaw_err_prev);

                vel_err_prev = vel_err;
                vel_err_int += vel_err/(double)CMD_RATE;
                vel_err_int = clamp_(vel_err, -VEL_ERR_INT_LIMIT, VEL_ERR_INT_LIMIT);
                yaw_err_prev = yaw_err;
                yaw_err_int += yaw_err/(double)CMD_RATE;
                yaw_err_int = clamp_(yaw_err, -YAW_ERR_INT_LIMIT, YAW_ERR_INT_LIMIT);
            }

            cmd_pub_.publish(cmd_);
            ros::spinOnce();
            cmd_rate_.sleep();
        }
    }

    if (equilibrium_time_.toSec() >= MIN_EQUILIBRIUM_TIME)
    {
        ilqr_msgs::BoolStamped timer_stop;
        timer_stop.header.stamp = ros::Time::now();
        timer_stop.data = false;
        timer_pub_.publish(timer_stop);
        ROS_INFO("Stayed on equilibrium for at least %d seconds. Quitting.", MIN_EQUILIBRIUM_TIME);
    }
}

double DriftCornering::clamp_(double val, double lower_limit, double upper_limit)
{
    if (upper_limit < lower_limit) {
        ROS_WARN("clamp function called with lower limit > upper limit. Swapping.");
        double tmp = upper_limit;
        upper_limit = lower_limit;
        lower_limit = tmp;
    }

    if (val > upper_limit) return upper_limit;
    else if (val < lower_limit) return lower_limit;
    else return val;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "drift_cornering_planner");
    
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    DriftCornering planner(nh, pnh);

    planner.spin();

    return 0;
}
