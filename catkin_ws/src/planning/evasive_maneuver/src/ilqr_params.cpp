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
    if (!getRequiredParam(nh, "vehicle/I_zz", _Iz)) return 1;
    if (!getRequiredParam(nh, "vehicle/wheelbase_f", _a)) return 1;
    if (!getRequiredParam(nh, "vehicle/wheelbase_r", _b)) return 1;
    _G_f = _m*G*_b/(_a+_b);
    _G_r = _m*G*_a/(_a+_b);
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

        if (strcmp(_params[i].name , "dt")          == 0) _params[i].data = &_dt; 
        if (strcmp(_params[i].name , "limThr")      == 0) _params[i].data = &_limThr[0]  ;
        if (strcmp(_params[i].name , "cu")          == 0) _params[i].data = &_cu[0]      ;
        if (strcmp(_params[i].name , "cdu")         == 0) _params[i].data = &_cdu[0]     ;
        if (strcmp(_params[i].name , "cf")          == 0) _params[i].data = &_cf[0]      ;
        if (strcmp(_params[i].name , "pf")          == 0) _params[i].data = &_pf[0]      ;
        if (strcmp(_params[i].name , "cx")          == 0) _params[i].data = &_cx[0]      ;
        if (strcmp(_params[i].name , "cdx")         == 0) _params[i].data = &_cdx[0]     ;
        if (strcmp(_params[i].name , "px")          == 0) _params[i].data = &_px[0]      ;
        if (strcmp(_params[i].name , "cdrift")      == 0) _params[i].data = &_cdrift     ;
        if (strcmp(_params[i].name , "lane_center") == 0) _params[i].data = &_lane_center;
        if (strcmp(_params[i].name , "lane_thres")  == 0) _params[i].data = &_lane_thres ;
        if (strcmp(_params[i].name , "croad")       == 0) _params[i].data = &_croad      ;
        if (strcmp(_params[i].name , "k_pos")       == 0) _params[i].data = &_k_pos      ;
        if (strcmp(_params[i].name , "k_vel")       == 0) _params[i].data = &_k_vel      ;
        if (strcmp(_params[i].name , "d_thres")     == 0) _params[i].data = &_d_thres    ;
        if (strcmp(_params[i].name , "m")           == 0) _params[i].data = &_m          ;
        if (strcmp(_params[i].name , "Iz")          == 0) _params[i].data = &_Iz         ;
        if (strcmp(_params[i].name , "a")           == 0) _params[i].data = &_a          ;
        if (strcmp(_params[i].name , "b")           == 0) _params[i].data = &_b          ;
        if (strcmp(_params[i].name , "G_f")         == 0) _params[i].data = &_G_f        ;
        if (strcmp(_params[i].name , "G_r")         == 0) _params[i].data = &_G_r        ;
        if (strcmp(_params[i].name , "c_x")         == 0) _params[i].data = &_c_x        ;
        if (strcmp(_params[i].name , "c_a")         == 0) _params[i].data = &_c_a        ;
        if (strcmp(_params[i].name , "mu")          == 0) _params[i].data = &_mu         ;
        if (strcmp(_params[i].name , "mu_s")        == 0) _params[i].data = &_mu_s       ;
        if (strcmp(_params[i].name , "limSteer")    == 0) _params[i].data = &_limSteer[0];

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

    if (num_set != N_CONST_PARAMS)
        ROS_ERROR("%d iLQR parameters not assigned.", N_CONST_PARAMS-num_set);
    else
        return 0;

}


    // typedef struct {
    //     param_t G_f;
    //     param_t G_r;
    //     param_t Iz;
    //     param_t a;
    //     param_t b;
    //     param_t c_a;
    //     param_t c_x;
    //     param_t cdrift;
    //     param_t cdu;
    //     param_t cdx;
    //     param_t cf;
    //     param_t croad;
    //     param_t cu;
    //     param_t cx;
    //     param_t d_thres;
    //     param_t dt;
    //     param_t k_pos;
    //     param_t k_vel;
    //     param_t lane_center;
    //     param_t lane_thres;
    //     param_t limSteer;
    //     param_t limThr;
    //     param_t m;
    //     param_t mu;
    //     param_t mu_s;
    //     param_t pf;
    //     param_t px;
    // } params_t;

    // params_t _params_;
    
    
    // _params_.G_f.name          = "G_f";
    // _params_.G_r.name          = "G_r";
    // _params_.Iz.name           = "Iz";
    // _params_.a.name            = "a";
    // _params_.b.name            = "b";
    // _params_.c_a.name          = "c_a";
    // _params_.c_x.name          = "c_x";
    // _params_.cdrift.name       = "cdrift";
    // _params_.cdu.name          = "cdu";
    // _params_.cdx.name          = "cdx";
    // _params_.cf.name           = "cf";
    // _params_.croad.name        = "croad";
    // _params_.cu.name           = "cu";
    // _params_.cx.name           = "cx";
    // _params_.d_thres.name      = "d_thres";
    // _params_.dt.name           = "dt";
    // _params_.k_pos.name        = "k_pos";
    // _params_.k_vel.name        = "k_vel";
    // _params_.lane_center.name  = "lane_center";
    // _params_.lane_thres.name   = "lane_thres";
    // _params_.limSteer.name     = "limSteer";
    // _params_.limThr.name       = "limThr";
    // _params_.m.name            = "m";
    // _params_.mu.name           = "mu";
    // _params_.mu_s.name         = "mu_s";
    // _params_.pf.name           = "pf";
    // _params_.px.name           = "px";