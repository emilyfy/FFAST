function X_dot = dynamics(X, U, params)
% Drifting dynamics developed based on 
% Dynamics And Control Of Drifting In Automobiles, Hindiyeh 2013
%
% Inputs: X, U, (optional) params
%  where  X = [x;y;psi;v_x;v_y;psi_dot]
%         U = [wheel_speed;steer_angle]
%         params struct (optional) stores parameters from iLQG,
%         member Obs stores obstacle speed
% Output: X_dot

global vehicle
global debug

if size(X,1) == 8
    if ~exist('params','var')
        obs_vx = 0;
        obs_vy = 1;
    else
        obs_vx = params.obs_vel(1);
        obs_vy = params.obs_vel(2);
    end
end

% ----------------------------------------
% ------States/Inputs Interpretation------
% ----------------------------------------
pos_x = X(1);
pos_y = X(2);
pos_yaw = wrapToPi(X(3));
v_x = X(4);
v_y = X(5);
yaw_rate = X(6);

cmd_vx = U(1);
delta = U(2);

% ----------------------------------------
% --------------Tire Dyanmics-------------
% ----------------------------------------

% find tire slip angle alpha

% no slip when stationary
if abs(v_x) < 0.01 && abs(v_y) < 0.01
    alpha_f = 0;
    alpha_r = 0;

% from page 58 of Hindiyeh, 2013
else
    alpha_f = atan2((v_y+vehicle.L_f*yaw_rate),v_x)-delta;
    alpha_r = atan2((v_y-vehicle.L_r*yaw_rate),v_x);
end

F_yf = tire_dyn_f(alpha_f);
[F_xr,F_yr] = tire_dyn_r(v_x, cmd_vx, alpha_r);

% ----------------------------------------
% ------------Vehicle Dyanmics------------
% ----------------------------------------

% find dX
T_z = vehicle.L_f*F_yf*cos(delta)-vehicle.L_r*F_yr;
ma_x = F_xr-F_yf*sin(delta);
ma_y = F_yf*cos(delta)+F_yr;

% without damping
%yaw_rate_dot = T_z/vehicle.I_z;
%v_x_dot = ma_x/vehicle.m + yaw_rate*v_y;
%v_y_dot = ma_y/vehicle.m - yaw_rate*v_x;
% with damping
yaw_rate_dot = T_z/vehicle.I_z -0.02*yaw_rate;
v_x_dot = ma_x/vehicle.m + yaw_rate*v_y -0.025*v_x;
v_y_dot = ma_y/vehicle.m - yaw_rate*v_x -0.025*v_y;

% translate to inertial frame
v = sqrt(v_x^2+v_y^2);
beta = atan2(v_y,v_x);
pos_x_dot = v*cos(beta+pos_yaw);
pos_y_dot = v*sin(beta+pos_yaw);

if size(X,1) == 6
    X_dot = [pos_x_dot;pos_y_dot;yaw_rate;v_x_dot;v_y_dot;yaw_rate_dot];
elseif size(X,1) == 8
    X_dot = [pos_x_dot;pos_y_dot;yaw_rate;v_x_dot;v_y_dot;yaw_rate_dot;obs_vx;obs_vy];
end

% set debug values
if isfield(debug,'F_xr')
    debug.F_xr = F_xr;
end
if isfield(debug,'F_yr')
    debug.F_yr = F_yr;
end
if isfield(debug,'F_yf')
    debug.F_yf = F_yf;
end
if isfield(debug,'alpha_f')
    debug.alpha_f = alpha_f;
end
if isfield(debug,'alpha_r')
    debug.alpha_r = alpha_r;
end
if isfield(debug,'beta')
    debug.beta = beta;
end
if isfield(debug,'cs')
    if (sign(delta) ~= sign (yaw_rate)) && (abs(delta) > 0.05)
        debug.cs = 1;
    else
        debug.cs = 0;
    end
end

end
