#ifndef _ILQR_PARAMS_H_
#define _ILQR_PARAMS_H_

//// REMEMBER TO CHANGE THIS IF THE iLQG_func.c FILE IS CHANGED!!!!!!! ////
//// THIS IS THE NUM OF CONSTANT PARAMS (EXCLUDE OBS_VEL)              ////
#define N_CONST_PARAMS 25
#define G 9.81

extern "C" {
  #include "iLQG.h"
  #include "iLQG_problem.h"
}

typedef struct {
  char *name;
  int size;
  double *data;
} param_t;

param_t* _params;

const char* _param_names[N_CONST_PARAMS] = {
"I_z",
"L_f",
"L_r",
"c_a",
"c_lane",
"c_obs",
"c_x",
"cdu",
"cf",
"cu",
"cx",
"dist_obs_thres",
"dt",
"goal",
"lane_center",
"lane_thres",
"limSteer",
"limThr",
"load_f",
"load_r",
"m",
"mu",
"mu_s",
"pf",
"px" };

double _dt             = 0.02                                       ;
double _limThr[2]      = {  -2.0,   4.0}                            ;
double _goal[6]        = {     5,     0,     0,     0,     0,     0};
double _cu[2]          = { 0.001,   0.0}                            ;
double _cdu[2]         = { 0.001, 0.006}                            ;
double _cf[6]          = {    18,     7,     5,    10,   0.1,   0.1};
double _pf[6]          = {  0.01,  0.01,   0.1,   0.1,   0.1,   0.1};
double _cx[3]          = {   2.5,   1.0,   1.2}                     ;
double _px[3]          = {  0.01,  0.01,   0.1}                     ;
double _dist_obs_thres = 0.6                                        ;
double _c_obs[3]       = {    15,   0.1,  0.01}                     ;
double _lane_center    = 0.0                                        ;
double _lane_thres     = 0.20                                       ;
double _c_lane         = 7                                          ;

double _load_f     ;
double _load_r     ;
double _I_z        ;
double _L_f        ;
double _L_r        ;
double _c_a        ;
double _c_x        ;
double _m          ;
double _mu         ;
double _mu_s       ;
double _limSteer[2];
double _limstr     ;


#endif