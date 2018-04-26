clear all
close all
clc

global vehicle
load('vehicle.mat')

% plot options
vis.show_traj_cog = 1;
vis.cog_color = 'g';
vis.show_traj_r = 1;
vis.rax_color = 'r';
vis.show_wheels = 1;
vis.color = 'k';
vis.follow = 1;
vis.x_clearance = 2;
vis.y_clearance = 2;
plt.dt = 0.02;
plt.titles = {'v_x','v_y','yaw rate'};

plt = init_plot(plt);
vis = init_vis(vis);

% initialize variables
%X = [0;0;0;0;0;0];
X = [0;0;0;1;0.216854;-0.772949];
cmd_vel = 4.253134;
steer = degtorad(15);

dt = 0.02;
U = [cmd_vel; steer];

global debug
debug.beta = 1;
debug.rts = '';
debug.cs = '';

tic

while 1
    X = state_transition(X,U,dt);
    
    vis = update_vis(vis, X, U);
    
    %fprintf('rw = %f delta = %f vxd = %f vyd = %f pd = %f \n', cmd_vel, radtodeg(steer), X(4), X(5), X(6))
    fprintf('rts = %d beta = %f cs = %d\n', debug.rts, radtodeg(debug.beta), debug.cs)
    plt.vals = [X(4),X(5),X(6)];
    plt = update_plot(plt);
end
