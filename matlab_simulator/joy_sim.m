% Adapted from
% Cyrus Liu
% the Robotics Institute, Carnegie Mellon University

% Vehicle Drifting Dynamics Simulation

clear all
close all
clc

global vehicle
load('vehicle.mat')

% --------Initialize Joystick--------
joy.throttle_axis = 5;
joy.steer_axis = 2;
joy.stop_button = 2;
joy.pause_button = 1;
joy.clear_button = 3;

joy.max_vel = 3;
joy.min_vel = -2;
joy.min_speed = 0.55;
joy.max_steer = pi/6;

joy.js = vrjoystick(1);

% --------Initialize Plots--------
vis.show_traj_cog = 1;
vis.cog_color = 'g';
vis.show_traj_r = 1;
vis.rax_color = 'r';
vis.show_wheels = 1;
vis.color = 'k';
vis.follow = 1;
vis.x_clearance = 2;
vis.y_clearance = 2;
plt.titles = {'v_x','v_y','yaw rate'};

plt = init_plot(plt);
vis = init_vis(vis);

% --------Initialize Variables--------
X = [0;0;0;0;0;0];
cmd_vel = 0;
steer = 0;

% debug
global debug
debug.beta = 1;
debug.rts = 0;
debug.cs = 0;

tic

while ~button(joy.js,joy.stop_button)
    
    % --------Use Joystick Input--------
    if axis(joy.js,joy.throttle_axis) < 0
        cmd_vel = axis(joy.js,joy.throttle_axis)^3*-1*joy.max_vel;
    else
        cmd_vel = axis(joy.js,joy.throttle_axis)^3*joy.min_vel;   
    end
    if abs(cmd_vel) < joy.min_speed
        cmd_vel = 0;
    end
    
    steer = joy.max_steer*-1*axis(joy.js,joy.steer_axis);
    
    if button(joy.js,joy.pause_button)
        pause()
    end
    
    if button(joy.js,joy.clear_button)
        vis = clear_traj(vis);
    end
    
    % ------Calculate Car Dynamics------
    dt = toc; tic
    %dt = 0.02;
    
    U = [cmd_vel; steer];
    X = state_transition(X,U,dt);

    vis = update_vis(vis, X, U);
    
    %fprintf('rw = %f delta = %f vxd = %f vyd = %f pd = %f \n', cmd_vel, radtodeg(steer), X(4), X(5), X(6))
    fprintf('rts = %d beta = %f cs = %d\n', debug.rts, radtodeg(debug.beta), debug.cs)
    plt.vals = [X(4),X(5),X(6)];
    plt = update_plot(plt);
end
