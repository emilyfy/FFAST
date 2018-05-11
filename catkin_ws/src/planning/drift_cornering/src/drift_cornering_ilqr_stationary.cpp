#include "drift_cornering_ilqr.h"

#include <math.h>
#include <boost/random.hpp>

#include <ros/ros.h>
#include <ilqr_msgs/IlqrInput.h>
#include <ilqr_msgs/IlqrOutput.h>
#include <ilqr_msgs/BoolStamped.h>

#define G 9.81

namespace stationary {
    #include "ilqr_params_stationary.h"
    // extern "C" {
        #include "iLQGStationary/iLQG.h"
        #include "iLQGStationary/iLQG_problem.h"
        #include "iLQGStationary/matMult.h"
    // }
}

using namespace stationary;

template <typename T>
inline bool getRequiredParam(const ros::NodeHandle& n, std::string name, T& value)
{
  if (n.getParam(name, value))
    return true;

  ROS_FATAL("iLQR: Parameter %s is required.", name.c_str());
  return false;
}

static tOptSet* Op_;

void iLQR::_setOpStationary(ros::NodeHandle nh, ros::NodeHandle pnh)
{
    // set values in Op_
    Op_ = (tOptSet *) malloc(sizeof(tOptSet));
    Op_->log_linesearch = NULL;
    Op_->log_z = NULL;
    Op_->log_cost = NULL;
    if (_assignParamsStationary(nh, pnh)) return;
    standard_parameters(Op_);
    Op_->n_hor = HORIZON_STATIONARY;
    Op_->max_iter = MAX_ITER;

    // allocate memory for Op_
    for (int i=0; i<NUMBER_OF_THREADS+1; i++) Op_->trajectories[i].t = (trajEl_t *) malloc(sizeof(trajEl_t)*HORIZON_STATIONARY);
    Op_->multipliers.t = (multipliersEl_t *) malloc(sizeof(multipliersEl_t)*(HORIZON_STATIONARY+1));
    Op_->nominal = (traj_t *) malloc(sizeof(traj_t));
    Op_->nominal->t = (trajEl_t *) malloc(sizeof(trajEl_t)*HORIZON_STATIONARY);
    for (int i=0; i<NUMBER_OF_THREADS; i++) {
        Op_->candidates[i] = (traj_t *) malloc(sizeof(traj_t));
        Op_->candidates[i]->t = (trajEl_t *) malloc(sizeof(trajEl_t)*HORIZON_STATIONARY);
    }

    // wait for equilibrium to be set and set goal in Op_
    ros::Duration(2).sleep();
    Op_->p[P_GOAL_IDX_STATIONARY] = &equilibrium_[2];
}

int iLQR::_assignParamsStationary(ros::NodeHandle nh, ros::NodeHandle pnh)
{
    // get values from rosparam server
    if (!getRequiredParam(nh, "vehicle/mass", _m)) return 1;
    if (!getRequiredParam(nh, "vehicle/I_zz", _I_z)) return 1;
    if (!getRequiredParam(nh, "vehicle/wheelbase_f", _L_f)) return 1;
    if (!getRequiredParam(nh, "vehicle/wheelbase_r", _L_r)) return 1;
    _load_f = _m*G*_L_r/(_L_f+_L_r);
    _load_r = _m*G*_L_f/(_L_f+_L_r);
    if (!getRequiredParam(nh, "vehicle/C_x", _c_x)) return 1;
    if (!getRequiredParam(nh, "vehicle/C_a", _c_a)) return 1;
    if (!getRequiredParam(nh, "vehicle/mu", _mu)) return 1;
    if (!getRequiredParam(nh, "vehicle/mu_s", _mu_s)) return 1;
    if (!getRequiredParam(nh, "vehicle/max_steering_angle", _limstr)) return 1;
    _limSteer[0] = -_limstr;
    _limSteer[1] = _limstr;

    // assign names and values to params
    if ( (_params = (param_t *) malloc(n_const_params*sizeof(param_t))) == NULL) {
        ROS_FATAL("Not enough memory.");
        return 1;
    }
    if ( (Op_->p = (double **) malloc(n_params*sizeof(double *))) == NULL) {
        ROS_FATAL("Not enough memory.");
        return 1;
    }
    
    int num_set = 0;

    for (int i=0; i<n_const_params; i++)
    {
        _params[i].name = (char *)_param_names[i];
        _params[i].size = 0;

        //// REMEMBER TO CHANGE THESE IF THE iLQG_func.c FILE IS CHANGED!!!!!!!  ////
        ////                     THESE ARE THE CONSTANT PARAMS                   ////

        if (strcmp(_params[i].name , "I_z"      ) == 0) _params[i].data = &_I_z           ;
        if (strcmp(_params[i].name , "L_f"      ) == 0) _params[i].data = &_L_f           ;
        if (strcmp(_params[i].name , "L_r"      ) == 0) _params[i].data = &_L_r           ;
        if (strcmp(_params[i].name , "c_a"      ) == 0) _params[i].data = &_c_a           ;
        if (strcmp(_params[i].name , "c_x"      ) == 0) _params[i].data = &_c_x           ;
        if (strcmp(_params[i].name , "cdu"      ) == 0) _params[i].data = &_cdu[0]        ;
        if (strcmp(_params[i].name , "cf"       ) == 0) _params[i].data = &_cf[0]         ;
        if (strcmp(_params[i].name , "cu"       ) == 0) _params[i].data = &_cu[0]         ;
        if (strcmp(_params[i].name , "cx"       ) == 0) _params[i].data = &_cx[0]         ;
        if (strcmp(_params[i].name , "dt"       ) == 0) _params[i].data = &_dt            ;
        if (strcmp(_params[i].name , "limSteer" ) == 0) _params[i].data = &_limSteer[0]   ;
        if (strcmp(_params[i].name , "limThr"   ) == 0) _params[i].data = &_limThr[0]     ;
        if (strcmp(_params[i].name , "load_f"   ) == 0) _params[i].data = &_load_f        ;
        if (strcmp(_params[i].name , "load_r"   ) == 0) _params[i].data = &_load_r        ;
        if (strcmp(_params[i].name , "m"        ) == 0) _params[i].data = &_m             ;
        if (strcmp(_params[i].name , "mu"       ) == 0) _params[i].data = &_mu            ;
        if (strcmp(_params[i].name , "mu_s"     ) == 0) _params[i].data = &_mu_s          ;
        if (strcmp(_params[i].name , "pf"       ) == 0) _params[i].data = &_pf[0]         ;
        if (strcmp(_params[i].name , "px"       ) == 0) _params[i].data = &_px[0]         ;

        for (int j=0; j<n_params; j++) {
            if (strcmp(_params[i].name,paramdesc[j]->name)==0) {
                _params[i].size = paramdesc[j]->size;
                Op_->p[j] = (double *) malloc(sizeof(double)*paramdesc[j]->size);
                memcpy(Op_->p[j], _params[i].data, _params[i].size*sizeof(double));
                num_set++;
                break;
            }
        }
        if (_params[i].size == 0)
            ROS_ERROR("iLQR parameter %s is not member of parameters struct.", _params[i].name);
    }

    if (num_set != n_const_params)
        ROS_ERROR("%d iLQR parameters not assigned.", n_const_params-num_set);
    else
        return 0;

}

void iLQR::_ilqrStationary(const ilqr_msgs::IlqrInput::ConstPtr& msg)
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
        u0_stationary_[2*i] = msg->remainingcommands[i].speed;
        u0_stationary_[2*i+1] = msg->remainingcommands[i].steering_angle;
    }
    for (;i<N_-1;i++) {
        u0_stationary_[2*i] = vel_dist(vel_gen);
        u0_stationary_[2*i+1] = steer_dist(steer_gen);
    }

    // assign Op_ values
    Op_->x0 = x0_;

    // outputs
    double x_new[N_*N_X], u_new[(N_-1)*N_U];
    ilqr_msgs::IlqrOutput output;
    output.commands.resize(N_-1);
    output.states.resize(N_);

    if (!init_opt(Op_)) {
        ROS_ERROR("Failed to initialize opt.");
        output.success = 0;
        output.cost = Op_->cost;
    } else {
        for (int c=0; c<N_-1; c++)
            for (int r=0; r<N_U; r++)
                Op_->nominal->t[c].u[r] = u0_stationary_[MAT_IDX(r, c, N_U)];
        
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
            for (int i=0; i<N_-1; i++) {
                output.commands[i].speed = u_new[MAT_IDX(0, i, N_U)];
                output.commands[i].steering_angle = u_new[MAT_IDX(1, i, N_U)];
            }
        }
    }

    output.header.stamp = ros::Time::now();
    ilqr_pub_.publish(output);
}