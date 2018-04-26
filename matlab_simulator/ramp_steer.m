clear all
close all
clc

global vehicle
load('vehicle.mat')

X = [0;0;0;0;0;0];
dt = 0.02;
cmd_vel = 3;

%{
vis.show_traj_cog = 1;
vis.show_wheels = 1;
vis.follow = 1;
vis.x_clearance = 2;
vis.y_clearance = 2;
plt.dt = dt;
plt.titles = {'v_x','\beta','yaw rate'};

plt = init_plot(plt);
vis = init_vis(vis);
%}

log.k = [];
log.alph_f = [];
log.alph_r = [];
log.Fyf = [];
log.Fyr = [];
log.Fxr = [];
x_prev = X(4:6);

global debug
debug.F_yr=0; debug.F_yf=0; debug.alpha_f=0; debug.alpha_r=0; debug.F_xr=0; debug.K=0;

for steer = -30:0
    U = [cmd_vel; degtorad(steer)];
    
    for i=0:dt:1
        %{
        vis = update_vis(vis, X, U);
        plt.vals = [X(4),atan2(X(5),X(4))*180/pi,X(6)];
        plt = update_plot(plt);
        %}

        x = X(4:6); u = U;
        %log.k = [log.k (u(1)-x(1))/abs(x(1))];
        %log.alph_f = [log.alph_f atan2((x(2)+vehicle.L_f*x(3)),x(1))-u(2)];
        %log.alph_r = [log.alph_r atan2((x(2)-vehicle.L_r*x(3)),x(1))];
        X = state_transition(X, U, dt);
        x_dot = (X(4:6) - x_prev)/dt;
        %log.Fxr = [log.Fxr -vehicle.m*x(3)*x(2)+vehicle.L_r*vehicle.m*sin(u(2))*x(3)*x(1)/(vehicle.L*cos(u(2)))];
        %log.Fyf = [log.Fyf vehicle.L_r/vehicle.L*vehicle.m*x(3)*x(1)/cos(u(2))];
        %log.Fyr = [log.Fyr vehicle.L_f/vehicle.L*vehicle.m*x(3)*x(1)];
        %log.Fxr = [log.Fxr vehicle.m*(x_dot(1)-x(3)*x(2))+(vehicle.I_z*x_dot(3)*sin(u(2))+vehicle.L_r*vehicle.m*sin(u(2))*(x_dot(2)+x(3)*x(1)))/(vehicle.L*cos(u(2)))];
        %log.Fyf = [log.Fyf (vehicle.I_z*x_dot(3)+vehicle.L_r*vehicle.m*(x_dot(2)+x_prev(3)*x_prev(1)))/(vehicle.L*cos(u(2)))];
        %log.Fyr = [log.Fyr (vehicle.L_f*vehicle.m*(x_dot(2)+x_prev(3)*x_prev(1))-vehicle.I_z*x_dot(3))/vehicle.L ];
        x_prev = X(4:6);
        log.k = [log.k debug.K];
        log.alph_f = [log.alph_f debug.alpha_f];
        log.alph_r = [log.alph_r debug.alpha_r];
        log.Fxr = [log.Fxr debug.F_xr];
        log.Fyf = [log.Fyf debug.F_yf];
        log.Fyr = [log.Fyr debug.F_yr];
    end
end

c = linspace(1,10,length(log.k(1:end)));

l.fig = figure();
l.ax = gca();
l.plt = scatter(log.k(1:end), log.Fxr(1:end),5,c,'filled');

f.fig = figure();
f.ax = gca();
f.plt = scatter(log.alph_f(1:end),log.Fyf(1:end),5,c,'filled');

r.fig = figure();
r.ax = gca();
r.plt = scatter(log.alph_r(1:end),log.Fyr(1:end),5,c,'filled');