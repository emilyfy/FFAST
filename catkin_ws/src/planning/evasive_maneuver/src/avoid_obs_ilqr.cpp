#include "avoid_obs_ilqr.h"

#include <ilqr_msgs/IlqrOutput.h>
extern "C" {
  #include "iLQG.h"
  #include "iLQG_problem.h"
}

iLQR::iLQR(ros::NodeHandle nh, ros::NodeHandle pnh) :
    vel_dist(1.5, 0.25), steer_dist(0.0, 0.1), N_(HORIZON+1)
{
    // set values in Op_
    Op_ = (tOptSet *) malloc(sizeof(tOptSet));
    Op_->log_linesearch = NULL;
    Op_->log_z = NULL;
    Op_->log_cost = NULL;
    if (assignParams(nh, pnh)) return;
    standard_parameters(Op_);
    Op_->n_hor = HORIZON;
    Op_->max_iter = MAX_ITER;

    // allocate memory for Op_
    for (int i=0; i<NUMBER_OF_THREADS+1; i++)
        Op_->trajectories[i].t = (trajEl_t *) malloc(sizeof(trajEl_t)*(N_-1));
    Op_->multipliers.t= (multipliersEl_t *) malloc(sizeof(multipliersEl_t)*N_);

    Op_->nominal = (traj_t *) malloc(sizeof(traj_t));
    Op_->nominal->t = (trajEl_t *) malloc(sizeof(trajEl_t)*(N_-1));
    for (int i=0; i<NUMBER_OF_THREADS; i++) {
        Op_->candidates[i] = (traj_t *) malloc(sizeof(traj_t));
        Op_->candidates[i]->t = (trajEl_t *) malloc(sizeof(trajEl_t)*(N_-1));
    }

    // setup ROS subscribers and publishers
    obs_sub_ = nh.subscribe("masked_obstacles", 1, &iLQR::obsCb, this);
    ilqr_sub_ = nh.subscribe("ilqr_input", 1, &iLQR::ilqrCb, this);
    ilqr_pub_ = nh.advertise<ilqr_msgs::IlqrOutput>("ilqr_output", 1);

    // set static / initial values
    x0_[8] = 3.0;
    x0_[9] = 0.0;
    obs_vel_[0] = 0.0;
    obs_vel_[1] = 0.0;
}

void iLQR::ilqrCb(const ilqr_msgs::IlqrInput::ConstPtr& msg)
{
    // get input values to iLQR planner
    x0_[0] = msg->state.pose.x;
    x0_[1] = msg->state.pose.y;
    x0_[2] = msg->state.pose.theta;
    x0_[3] = msg->state.twist.x;
    x0_[4] = msg->state.twist.y;
    x0_[5] = msg->state.twist.theta;
    x0_[6] = msg->previouscommand.speed;
    x0_[7] = msg->previouscommand.steering_angle;

    int i;
    for (i=0;i<msg->remainingcommands.size();i++) {
        u0_[2*i] = msg->remainingcommands[i].speed;
        u0_[2*i+1] = msg->remainingcommands[i].steering_angle;
    }
    for (;i<HORIZON;i++) {
        u0_[2*i] = vel_dist(vel_gen);
        u0_[2*i+1] = steer_dist(steer_gen);
    }

    // assign Op_ values
    Op_->x0 = x0_;
    Op_->p[P_OBS_VEL_IDX] = obs_vel_;
    Op_->p[P_GOAL_IDX][0]-x0_[0] > CHANGE_CF_DIST ? Op_->p[P_CF_IDX] = cf_bef_goal_ : Op_->p[P_CF_IDX] = cf_aft_goal_;

    // outputs
    double x_new[N_*N_X], u_new[HORIZON*N_U];
    ilqr_msgs::IlqrOutput output;
    output.commands.resize(HORIZON);
    output.states.resize(N_);

    if (!init_opt(Op_)) {
        ROS_ERROR("Failed to initialize opt.");
        output.success = 0;
        output.cost = Op_->cost;
    } else {
        for (int c=0; c<N_-1; c++)
            for (int r=0; r<N_U; r++)
                Op_->nominal->t[c].u[r] = u0_[MAT_IDX(r, c, N_U)];
        
        if (!forward_pass(Op_->candidates[0], Op_, 0.0, &Op_->cost, 0)) {
            ROS_ERROR("Forward pass failed");
            output.success = 0;
            output.cost = Op_->cost;
        }
        
        else
        {
            makeCandidateNominal(Op_, 0);
            // ROS_DEBUG("Starting iLQG");
            output.success = iLQG(Op_);
            printf("iLQG %s.\n", output.success?"success":"fail");

            for (int c=0; c<N_-1; c++)
                for (int r=0; r<N_X; r++)
                    x_new[MAT_IDX(r, c, N_X)] = Op_->nominal->t[c].x[r];
            for (int r=0; r<N_X; r++)
                x_new[MAT_IDX(r, N_-1, N_X)] = Op_->nominal->f.x[r];
            
            for (int c=0; c<N_-1; c++)
                for (int r=0; r<N_U; r++)
                    u_new[MAT_IDX(r, c, N_U)] = Op_->nominal->t[c].u[r];
            
            output.cost = Op_->cost;
            
            // copy to ROS message
            for (int i=0; i<N_; i++) {
                output.states[i].pose.x = x_new[MAT_IDX(0, i, N_X)];
                output.states[i].pose.y = x_new[MAT_IDX(1, i, N_X)];
                output.states[i].pose.theta = x_new[MAT_IDX(2, i, N_X)];
                output.states[i].twist.x = x_new[MAT_IDX(3, i, N_X)];
                output.states[i].twist.y = x_new[MAT_IDX(4, i, N_X)];
                output.states[i].twist.theta = x_new[MAT_IDX(5, i, N_X)];
            }
            for (int i=0; i<HORIZON; i++) {
                output.commands[i].speed = u_new[MAT_IDX(0, i, N_U)];
                output.commands[i].steering_angle = u_new[MAT_IDX(1, i, N_U)];
            }
        }
    }

    output.header.stamp = ros::Time::now();
    ilqr_pub_.publish(output);
}

void iLQR::obsCb(const obstacle_detector::Obstacles::ConstPtr& msg)
{
    x0_[8] = msg->circles[0].center.x;
    x0_[9] = msg->circles[0].center.y;
    obs_vel_[0] = msg->circles[0].velocity.x;
    obs_vel_[1] = msg->circles[0].velocity.y;


}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "avoid_obs_ilqr");

    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    iLQR ilqr(nh, pnh);

    ros::spin();

    return 0;
}