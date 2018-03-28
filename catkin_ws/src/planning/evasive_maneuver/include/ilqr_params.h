#ifndef _ILQR_PARAMS_H_
#define _ILQR_PARAMS_H_

//// REMEMBER TO CHANGE THIS IF THE iLQG_func.c FILE IS CHANGED!!!!!!! ////
//// THIS IS THE NUM OF CONSTANT PARAMS (EXCLUDE OBS, XDES)            ////
#define N_CONST_PARAMS 27
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
"G_f",
"G_r",
"Iz",
"a",
"b",
"c_a",
"c_x",
"cdrift",
"cdu",
"cdx",
"cf",
"croad",
"cu",
"cx",
"d_thres",
"dt",
"k_pos",
"k_vel",
"lane_center",
"lane_thres",
"limSteer",
"limThr",
"m",
"mu",
"mu_s",
"pf",
"px" };

double _dt          = 0.02;
double _limThr[2]   = {  -2.0,   4.0}                            ;
double _cu[2]       = {   0.1,   0.1}                            ;
double _cdu[2]      = {  0.04,  0.06}                            ;
double _cf[6]       = {    15,     8,     5,    10,   0.1,   0.1};
double _pf[6]       = {  0.01,  0.01,   0.1,   0.1,   0.1,   0.1};
double _cx[3]       = {   2.5,   0.5,   3.3}                     ;
double _cdx[3]      = { 0.001, 0.005, 0.002}                     ;
double _px[3]       = {  0.01,  0.01,   0.1}                     ;
double _cdrift      = -0.00;
double _lane_center = 0.34;
double _lane_thres  = 0.30;
double _croad       = 15;
double _k_pos       = 14;
double _k_vel       = -5;
double _d_thres     = 0.3;

double _m          ;
double _Iz         ;
double _a          ;
double _b          ;
double _G_f        ;
double _G_r        ;
double _c_x        ;
double _c_a        ;
double _mu         ;
double _mu_s       ;
double _limSteer[2];
double _limstr     ;


#endif