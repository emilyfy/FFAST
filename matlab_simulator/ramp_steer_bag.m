clear all
load('../mats/rs.mat')
load('vehicle.mat')

last_cmd_idx = 2;

start_time = odom{100,'rosbag_recv_time'};

k = [];
af = [];
ar = [];
fx = [];
ff = [];
fr = [];

%x_prev = [0 0 0];
%prev_time = odom{49,'rosbag_recv_time'} - start_time;

for odom_idx = 50:size(odom,1)-50
    curr_time = odom{odom_idx,'rosbag_recv_time'} - start_time;
    
    % find corresponding messages
    for i = last_cmd_idx:size(cmd,1)
        cmd_time = cmd{i,'rosbag_recv_time'};
        if cmd_time > curr_time
            last_cmd_idx = i;
            break
        end
    end
    cmd_idx = last_cmd_idx-1;
    
    alpha_f = atan2((v_y+vehicle.L_f*yaw_rate),v_x)-cmd_steer;
    alpha_r = atan2((v_y-vehicle.L_r*yaw_rate),v_x);
    
    % get values
    v_x = odom{odom_idx,'twist_twist_linear_x'};
    v_y = odom{odom_idx,'twist_twist_linear_y'};
    yaw_rate = odom{odom_idx,'twist_twist_angular_z'};
    cmd_sp = cmd{cmd_idx,'drive_speed'};
    cmd_steer = cmd{cmd_idx,'drive_steering_angle'};
    
    F_xr = -vehicle.m*yaw_rate*v_y+vehicle.L_r*vehicle.m*sin(cmd_steer)*yaw_rate*v_x/(vehicle.L*cos(cmd_steer));
    F_yf = vehicle.L_r/vehicle.L*vehicle.m*yaw_rate*v_x/cos(cmd_steer);
    F_yr = vehicle.L_f/vehicle.L*vehicle.m*yaw_rate*v_x;
    
    %x = [v_x v_y yaw_rate];
    %dt = curr_time - prev_time;
    %x_dot = (x - x_prev)/dt;
    %F_xr = vehicle.m*(x_dot(1)-x(3)*x(2))+...
    %       (vehicle.I_z*x_dot(3)*sin(cmd_steer)+vehicle.L_r*vehicle.m*sin(cmd_steer)*(x_dot(2)+x(3)*x(1)))/(vehicle.L*cos(cmd_steer));
    %F_yf = (vehicle.I_z*x_dot(3)+vehicle.L_r*vehicle.m*(x_dot(2)+x_prev(3)*x_prev(1)))/(vehicle.L*cos(cmd_steer));
    %F_yr = (vehicle.L_f*vehicle.m*(x_dot(2)+x_prev(3)*x_prev(1))-vehicle.I_z*x_dot(3))/vehicle.L;
    %x_prev = [v_x v_y yaw_rate];
    %prev_time = curr_time;
    
    k = [k (cmd_sp-v_x)/v_x];
    af = [af alpha_f];
    ar = [ar alpha_r];
    fx = [fx F_xr];
    ff = [ff F_yf];
    fr = [fr F_yr];
end

c = linspace(1,10,length(k(1:end)));

l.fig = figure();
l.ax = gca();
l.plt = scatter(k(1:end),fx(1:end),5,c,'filled');

f.fig = figure();
f.ax = gca();
f.plt = scatter(af(1:end),ff(1:end),5,c,'filled');

r.fig = figure();
r.ax = gca();
r.plt = scatter(ar(1:end),fr(1:end),5,c,'filled');