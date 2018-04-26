function vehicle_params()
% Sets the parameters on the global struct vehicle
% Same values as vehicle.mat

% The parameters used in this example refer to a 1/10 scale RC car
% The model is expected to work for full scale vehicle as well

% ----------------------------------------
% --------------Model Params--------------
% ----------------------------------------

global vehicle

vehicle.L = 0.257;          % wheelbase (m)
vehicle.L_f = 0.115;        % CoG to front axle (m)
vehicle.L_r = 0.142;        % CoG to rear axle (m)
vehicle.tw = 0.165;         % trackwidth (m)

vehicle.wheel_dia = 0.0323; % wheel diameter (m)
vehicle.wheel_w = 0.025;    % wheel width (m)

vehicle.m = 2.596;          % mass (kg)
g = 9.81;                   % gravity const (m/s^2)

vehicle.load_f = vehicle.m*g*vehicle.L_r/vehicle.L;
vehicle.load_r = vehicle.m*g*vehicle.L_f/vehicle.L;

vehicle.C_x = 103.94;       % longitudinal stiffness (N)
vehicle.C_alpha = 56.4;     % cornering stiffness (N)
vehicle.I_z = 0.0558;       % rotation inertia (kgm^2)
vehicle.mu = 1.37;          % friction coefficient
vehicle.mu_slide = 1.96;    % sliding friction coefficient

end
