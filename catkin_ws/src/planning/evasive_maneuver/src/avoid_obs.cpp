#include "avoid_obs.h"

#define GOAL_TOLERANCE 0.5
#define CMD_RATE        50

#define VEL_KP   0.001
#define VEL_KI   0
#define VEL_KD   0
#define YAW_KP   0.001
#define YAW_KI   0
#define YAW_KD   0
#define VEL_ERR_INT_LIMIT 100
#define YAW_ERR_INT_LIMIT 100

AvoidObs::AvoidObs(ros::NodeHandle nh, ros::NodeHandle pnh) : 
    cmd_on_plan_(0), cmd_rate_(CMD_RATE), goal_set_(false), dist_to_goal_(5.0)
{
    // setup publishers and subscribers
    state_sub_  = nh.subscribe("ekf_localization/odom", 10, &AvoidObs::stateCb, this);
    goal_sub_ = nh.subscribe("goal", 1, &AvoidObs::goalCb, this);
    ilqr_sub_ = nh.subscribe("ilqr_output", 1, &AvoidObs::ilqrCb, this);
    goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>("goal", 1);
    cmd_pub_ = nh.advertise<ackermann_msgs::AckermannDriveStamped>("commands/cmd_vel", 5);
    ilqr_pub_ = nh.advertise<evasive_maneuver::IlqrInput>("ilqr_input", 1);
    path_pub_ = nh.advertise<nav_msgs::Path>("ilqr_path", 1);

    // wait for iLQR node and localization to start
    ros::Duration(5).sleep();
    // default goal
    if (!goal_set_) {
        geometry_msgs::PoseStamped goal;
        goal.header.stamp = ros::Time::now();
        goal.header.frame_id = "map";
        goal.pose.position.x = 5.0;
        goal.pose.position.y = 0.0;
        goal.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);
        goal_pub_.publish(goal);
    }
}

void AvoidObs::stateCb(const nav_msgs::Odometry::ConstPtr& msg)
{
    state_.pose.x = msg->pose.pose.position.x;
    state_.pose.y = msg->pose.pose.position.y;
    state_.pose.theta = tf::getYaw(msg->pose.pose.orientation);
    state_.twist.x = msg->twist.twist.linear.x;
    state_.twist.y = msg->twist.twist.linear.y;
    state_.twist.theta = msg->twist.twist.angular.z;

    // dist_to_goal_ = sqrt( pow(state_.pose.x-goal_.x,2.0) + pow(state_.pose.y-goal_.y,2.0) );
    dist_to_goal_ = fabs(state_.pose.x-goal_.x);
}

void AvoidObs::goalCb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (!goal_set_) {
        goal_.x = msg->pose.position.x;
        goal_.y = msg->pose.position.y;
        goal_.theta = tf::getYaw(msg->pose.orientation);

        goal_set_ = true;

        // dist_to_goal_ = sqrt( pow(state_.pose.x-goal_.x,2.0) + pow(state_.pose.y-goal_.y,2.0) );
        dist_to_goal_ = fabs(state_.pose.x-goal_.x);

        // publish the first iLQR input
        ilqr_input_.header.stamp = ros::Time::now();
        ilqr_input_.state = state_;
        ilqr_input_.remainingcommands.clear();
        ilqr_input_.goal = goal_;
        ilqr_pub_.publish(ilqr_input_);
    }
    else
        ROS_ERROR("Error setting goal: current goal not reached yet.");
}

void AvoidObs::ilqrCb(const evasive_maneuver::IlqrOutput &msg)
{
    ilqr_input_.header.stamp = ros::Time::now();
    ilqr_input_.state = state_;
    ilqr_input_.previouscommand = cmd_.drive;
    ilqr_input_.remainingcommands.clear();
    for (int i=cmd_on_plan_; i<plan_.commands.size(); i++)
        ilqr_input_.remainingcommands.push_back(plan_.commands[i]);
    ilqr_input_.goal = goal_;
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

void AvoidObs::spin(void)
{
    while (ros::ok())
    {
        while (ros::ok() && goal_set_ && dist_to_goal_ > GOAL_TOLERANCE)
        {
            cmd_.header.stamp = ros::Time::now();

            if (cmd_on_plan_ >= plan_.commands.size()) {
                ROS_ERROR("Planner reached end of iLQR command sequence.");
                cmd_.drive.speed = 0.0;
                cmd_.drive.steering_angle = 0.0;
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

        if (goal_set_) {
            ROS_INFO("Reached goal.");
            goal_set_ = false;
        }

        ros::spinOnce();
    }
}

double AvoidObs::clamp_(double val, double lower_limit, double upper_limit)
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
    ros::init(argc, argv, "avoid_obs_planner");
    
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    AvoidObs planner(nh, pnh);

    planner.spin();

    return 0;
}
