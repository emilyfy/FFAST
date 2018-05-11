#ifndef _ILQR_PARAMS_STATIONARY_H_
#define _ILQR_PARAMS_STATIONARY_H_

// extern "C" {
  #include "iLQGStationary/iLQG.h"
  #include "iLQGStationary/iLQG_problem.h"
// }

//// REMEMBER TO CHANGE THIS IF THE iLQG_func.c FILE IS CHANGED!!!!!!! ////
//// THIS IS THE NUM OF CONSTANT PARAMS (EXCLUDE GOAL)                 ////
const int n_const_params = 19;

typedef struct {
  char *name;
  int size;
  double *data;
} param_t;

param_t* _params;

const char* _param_names[n_const_params] = {
"I_z"      ,
"L_f"      ,
"L_r"      ,
"c_a"      ,
"c_x"      ,
"cdu"      ,
"cf"       ,
"cu"       ,
"cx"       ,
"dt"       ,
"limSteer" ,
"limThr"   ,
"load_f"   ,
"load_r"   ,
"m"        ,
"mu"       ,
"mu_s"     ,
"pf"       ,
"px"       };

double _dt             = 0.02                                       ;
double _limThr[2]      = {  -3.0,   6.0}                            ;
double _cu[2]          = { 0.001, 0.001}                            ;
double _cdu[2]         = {0.0001,0.0006}                            ;
double _cf[3]          = {    10,    10,    10}                     ;
double _pf[3]          = {   0.1,   0.1,   0.1}                     ;
double _cx[6]          = {     1,     1,     0,     5,     5,     5};
double _px[6]          = {  0.01,  0.01,     0,   0.1,   0.1,   0.1};

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