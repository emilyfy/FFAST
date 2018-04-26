clear all
close all
clc

global vehicle debug
load('vehicle.mat')

% --------Initialize ROS--------
try
    rosinit('10.42.0.2');
end
ros_dev = rosdevice('tegra-ubuntu','nvidia','nvidia');

% --------Initialize Joystick--------
global joy
joy.throttle_axis = 5;
joy.steer_axis = 1;
joy.brake_button = 2;
joy.activate_button = 1;
joy.reset_button = 3;
joy.matlab_stop_button = 4;
joy.estop_button = 6;

joy.max_vel = 3;
joy.min_vel = -2;
joy.min_speed = 0.55;
joy.max_steer = pi/6;
joy.stop = 0;
joy.reset = 0;
joy.active = 0;

sub_pose = rossubscriber('/ekf_localization/odom',@poseCallback);
sub_cmd = rossubscriber('/joy',@joyCallback);
%sub_vel = rossubscriber('/commands/motor/speed',@velCallback);
%sub_steer = rossubscriber('/commands/servo/position',@steerCallback);
global cmd_vel steer ros_X

sim_X = [0;0;0;0;0;0];
ros_X = [0;0;0];
cmd_vel = 0;
steer = 0;

sim_vis.color = 'b';
sim_vis.show_traj_cog = 1;
sim_vis.cog_color = 'c';
sim_vis.show_traj_r = 0;
sim_vis.show_wheels = 1;

ros_vis.color = 'k';
ros_vis.show_traj_cog = 1;
ros_vis.cog_color = 'm';
ros_vis.show_traj_r = 0;
ros_vis.show_wheels = 1;

sim_vis.follow = 1;
ros_vis.follow = 0;
sim_vis.x_clearance = 8;
sim_vis.y_clearance = 4;

sim_vis = init_vis(sim_vis);
ros_vis = init_vis(ros_vis);

plt.titles = {'rw','delta'};
plt = init_plot(plt);

tic

while ~joy.stop    
    
    % ------Calculate Car Dynamics------
    dt = toc; tic
    U = [cmd_vel; steer];
    sim_X = state_transition(sim_X,U,dt);
    
    if joy.reset
        sim_X = [0;0;0;0;0;0];
        sim_vis = clear_traj(sim_vis);
        ros_vis = clear_traj(ros_vis);
    end
    
    sim_vis = update_vis(sim_vis, sim_X, U);
    ros_vis = update_vis(ros_vis, ros_X, U);

    plt.vals = [cmd_vel,radtodeg(steer)];
    plt = update_plot(plt);
end

rosshutdown