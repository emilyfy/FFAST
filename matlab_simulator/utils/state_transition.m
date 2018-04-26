function [ X_new ] = state_transition( X, U, dt, params)
% Discrete state transition function
% X_t+1 = state_transition(X_t, U, dt)
% or X_t+1 = state_transition(X_t, U, dt, params)
% params struct (optional) stores parameters from iLQG,
% member Obs stores obstacle speed

if ~exist('params','var')
    X_dot = dynamics(X, U);
    X_new = X + X_dot.*dt;
else
    global vehicle

    vehicle.m = params.m;
    vehicle.L = params.L_f + params.L_r;
    vehicle.L_f = params.L_f;
    vehicle.L_r = params.L_r;
    vehicle.load_f = params.load_f;
    vehicle.load_r = params.load_r;
    
    vehicle.C_x = params.c_x;
    vehicle.C_alpha = params.c_a;
    vehicle.I_z = params.I_z;
    vehicle.mu = params.mu;
    vehicle.mu_s = params.mu_s;

    X_dot = dynamics(X, U, params);
    X_new = X + X_dot.*dt;
end

end
