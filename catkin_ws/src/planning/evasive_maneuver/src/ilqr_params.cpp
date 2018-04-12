#include "avoid_obs_ilqr.h"
#include "ilqr_params.h"


template <typename T>
inline bool getRequiredParam(const ros::NodeHandle& n, std::string name, T& value)
{
  if (n.getParam(name, value))
    return true;

  ROS_FATAL("iLQR: Parameter %s is required.", name.c_str());
  return false;
}

int iLQR::assignParams(ros::NodeHandle nh, ros::NodeHandle pnh)
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
    if ( (_params = (param_t *) malloc(N_CONST_PARAMS*sizeof(param_t))) == NULL) {
        ROS_FATAL("Not enough memory.");
        return 1;
    }
    if ( (Op_->p = (double **) malloc(n_params*sizeof(double *))) == NULL) {
        ROS_FATAL("Not enough memory.");
        return 1;
    }
    
    int num_set = 0;

    for (int i=0; i<N_CONST_PARAMS; i++)
    {
        _params[i].name = (char *)_param_names[i];
        _params[i].size = 0;

        //// REMEMBER TO CHANGE THESE IF THE iLQG_func.c FILE IS CHANGED!!!!!!!  ////
        ////                     THESE ARE THE CONSTANT PARAMS                   ////

        if (strcmp(_params[i].name , "I_z")            == 0) _params[i].data = &_I_z           ;
        if (strcmp(_params[i].name , "L_f")            == 0) _params[i].data = &_L_f           ;
        if (strcmp(_params[i].name , "L_r")            == 0) _params[i].data = &_L_r           ;
        if (strcmp(_params[i].name , "c_a")            == 0) _params[i].data = &_c_a           ;
        if (strcmp(_params[i].name , "c_lane")         == 0) _params[i].data = &_c_lane        ;
        if (strcmp(_params[i].name , "c_obs")          == 0) _params[i].data = &_c_obs[0]      ;
        if (strcmp(_params[i].name , "c_x")            == 0) _params[i].data = &_c_x           ;
        if (strcmp(_params[i].name , "cdu")            == 0) _params[i].data = &_cdu[0]        ;
        if (strcmp(_params[i].name , "cf")             == 0) _params[i].data = &_cf[0]         ;
        if (strcmp(_params[i].name , "cu")             == 0) _params[i].data = &_cu[0]         ;
        if (strcmp(_params[i].name , "cx")             == 0) _params[i].data = &_cx[0]         ;
        if (strcmp(_params[i].name , "dist_obs_thres") == 0) _params[i].data = &_dist_obs_thres;
        if (strcmp(_params[i].name , "dt")             == 0) _params[i].data = &_dt            ;
        if (strcmp(_params[i].name , "goal")           == 0) _params[i].data = &_goal[0]       ;
        if (strcmp(_params[i].name , "lane_center")    == 0) _params[i].data = &_lane_center   ;
        if (strcmp(_params[i].name , "lane_thres")     == 0) _params[i].data = &_lane_thres    ;
        if (strcmp(_params[i].name , "limSteer")       == 0) _params[i].data = &_limSteer[0]   ;
        if (strcmp(_params[i].name , "limThr")         == 0) _params[i].data = &_limThr[0]     ;
        if (strcmp(_params[i].name , "load_f")         == 0) _params[i].data = &_load_f        ;
        if (strcmp(_params[i].name , "load_r")         == 0) _params[i].data = &_load_r        ;
        if (strcmp(_params[i].name , "m")              == 0) _params[i].data = &_m             ;
        if (strcmp(_params[i].name , "mu")             == 0) _params[i].data = &_mu            ;
        if (strcmp(_params[i].name , "mu_s")           == 0) _params[i].data = &_mu_s          ;
        if (strcmp(_params[i].name , "pf")             == 0) _params[i].data = &_pf[0]         ;
        if (strcmp(_params[i].name , "px")             == 0) _params[i].data = &_px[0]         ;

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

    // set variables for cf
    cf_bef_goal_[0] = _cf[0];
    cf_aft_goal_[0] = 0;
    for (int i=1; i<6; i++) {
        cf_bef_goal_[i] = 0;
        cf_aft_goal_[i] = _cf[i];
    }

    if (num_set != N_CONST_PARAMS)
        ROS_ERROR("%d iLQR parameters not assigned.", N_CONST_PARAMS-num_set);
    else
        return 0;

}