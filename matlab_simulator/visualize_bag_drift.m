clear all
load('../mats/1.mat')

global vehicle
load('vehicle.mat')

% comparison with ilqr working in matlab
p.dt= 0.02;

p.m = vehicle.m;
p.L_r = vehicle.L_r;
p.L_f = vehicle.L_f;
p.load_f = vehicle.load_f;
p.load_r = vehicle.load_r;

p.c_x = vehicle.C_x;
p.c_a = vehicle.C_alpha;
p.I_z = vehicle.I_z;
p.mu = vehicle.mu;
p.mu_s = vehicle.mu_slide;

p.limThr = [-3 6];
p.limSteer = [-0.6  0.6];

p.cu = 1e-3*[1 1];
p.cdu = 1e-4*[1 6];

p.cx = [1 1 0 5 5 5];
p.px = [.01 .01 0 .1 .1 .1];

p.cf = [10 10 10];
p.pf = [0.1 0.1 0.1];

sol = [3.90890371627950 , 0.174532925199433  , 1.50000000000000 ,  1.45098618681538  , -5.86344940495027];
p.goal = [sol(3);sol(4);sol(5)];

Op.tolFun = 1e-7;
Op.lambdaMax = 1e10;
Op.max_iter = 500;
T = 50;

seq.succ_mat = [];
seq.U_mat = [];
seq.cost_mat = [];
seq.path_mat = [];

seq.succ_ros = [];
seq.U_ros = [];
seq.cost_ros = [];
seq.path_ros = [];

seq.X = [];
seq.U = [];

%% calculate

last_cmd_idx = 2;
last_input_idx = 2;
last_output_idx = 2;

start_time = timer{1,'rosbag_recv_time'};

% find starting odom msg
for i = 1:size(odom,1)
    odom_time = odom{i,'rosbag_recv_time'} - start_time;
    if odom_time > 0
        starting_odom_idx = i;
        break
    end
end

times = [];
for odom_idx = starting_odom_idx:size(odom,1)-20
    curr_time = odom{odom_idx,'rosbag_recv_time'} - start_time;
    times = [times curr_time];
    
    % find corresponding messages
    
    for i = last_cmd_idx:size(cmd,1)
        cmd_time = cmd{i,'rosbag_recv_time'} - start_time;
        if cmd_time > curr_time
            last_cmd_idx = i;
            break
        end
    end
    cmd_idx = last_cmd_idx-1;
    
    for i = last_input_idx:size(ilqr_input,1)
        input_time = ilqr_input{i,'rosbag_recv_time'} - start_time;
        if input_time > curr_time
            last_input_idx = i;
            break
        end
    end
    input_idx = last_input_idx-1;
    
    for i = last_output_idx:size(ilqr_output,1)
        output_time = ilqr_output{i,'rosbag_recv_time'} - start_time;
        if output_time > curr_time
            last_output_idx = i;
            break
        end
    end
    output_idx = last_output_idx-1;
    
    % get values
    pos_x = odom{odom_idx,'pose_pose_position_x'};
    pos_y = odom{odom_idx,'pose_pose_position_y'};
    e = quat2eul([odom{odom_idx,'pose_pose_orientation_w'},odom{odom_idx,'pose_pose_orientation_x'},odom{odom_idx,'pose_pose_orientation_y'},odom{odom_idx,'pose_pose_orientation_z'}]);
    yaw = e(1);
    
    v_x = odom{odom_idx,'twist_twist_linear_x'};
    v_y = odom{odom_idx,'twist_twist_linear_y'};
    yaw_rate = odom{odom_idx,'twist_twist_angular_z'};
    
    prev_cmd_sp = cmd{cmd_idx,'drive_speed'};
    prev_cmd_steer = cmd{cmd_idx,'drive_steering_angle'};
    
    % ilqr in matlab
    x0 = [pos_x;pos_y;yaw;
          v_x;v_y;yaw_rate;
          prev_cmd_sp;prev_cmd_steer];
    
    u0 = [];
    n = size(ilqr_input{input_idx,'remainingcommands'}{1},2);
    for i=1:n
        u0 = [u0 [ilqr_input{input_idx,'remainingcommands'}{1}{i}.speed; ilqr_input{input_idx,'remainingcommands'}{1}{i}.steering_angle] ];
    end
    u0 = [u0 [0.5*randn(1,T-n)+sol(1) ; 0.3*randn(1,T-n)] ];
    
    [success, xx, uu, cost] = iLQGDriftCorneringStationary(x0, u0, p, Op);

    % comparing matlab output and ros msg output
    seq.succ_mat = [seq.succ_mat success];
    seq.path_mat = cat(3,seq.path_mat,xx);
    seq.U_mat = cat(3,seq.U_mat,uu);
    seq.cost_mat = [seq.cost_mat cost];

    for i=1:T+1
        xx_ros(1,i) = ilqr_output{output_idx,'states'}{i}.pose.x;
        xx_ros(2,i) = ilqr_output{output_idx,'states'}{i}.pose.y;
        xx_ros(3,i) = ilqr_output{output_idx,'states'}{i}.pose.theta;
        xx_ros(4,i) = ilqr_output{output_idx,'states'}{i}.twist.x;
        xx_ros(5,i) = ilqr_output{output_idx,'states'}{i}.twist.y;
        xx_ros(6,i) = ilqr_output{output_idx,'states'}{i}.twist.theta;
    end
    for i=1:T
        uu_ros(1,i) = ilqr_output{output_idx,'commands'}{i}.speed;
        uu_ros(2,i) = ilqr_output{output_idx,'commands'}{i}.steering_angle;
    end
    seq.succ_ros = [seq.succ_ros ilqr_output{output_idx,'success'}];
    seq.path_ros = cat(3,seq.path_ros,xx_ros);
    seq.U_ros = cat(3,seq.U_ros,uu_ros);
    seq.cost_ros = [seq.cost_ros ilqr_output{output_idx,'cost'}];
    
    seq.X = [seq.X [pos_x;pos_y;yaw;v_x;v_y;yaw_rate] ];
    seq.U = [seq.U [prev_cmd_sp;prev_cmd_steer] ];
end

dt = [];
for i=2:size(times,2)
    dt = [dt times(i)-times(i-1)];
end
dt = mean(dt);

fprintf('matlab total succ: %d ave cost: %f\n', sum(seq.succ_mat), mean(seq.cost_mat));
fprintf('ros total succ: %d ave cost: %f\n', sum(seq.succ_ros), mean(seq.cost_ros));

%% plot

close all
rvis.show_wheels = 1;
rvis.path_color = 'g';
rvis.keepfig = 1;
rvis.show_grid = 1;
rvis.follow = 1;
rvis.x_clearance = 2;
rvis.y_clearance = 2;
rvis = init_vis(rvis);
mvis.show_wheels = 1;
mvis.path_color = 'b';
mvis.keepfig = 1;
mvis = init_vis(mvis);
mvis.m = [];
for i=1:size(seq.X,2)
    %mvis.txt = text(rvis.ax,seq.X(1,i),seq.X(2,i)-1.8,['time =' num2str(times(:,i)) ' s'],...
    %           'HorizontalAlignment','center');
    mvis = update_vis(mvis, seq.X(:,i), seq.U(:,i), seq.path_mat(:,:,i));
    rvis = update_vis(rvis, seq.X(:,i), seq.U(:,i), seq.path_ros(:,:,i));
    %delete(mvis.txt)
end

plt.titles = {'v_x','\beta','yaw rate'};
plt.dt = dt;
plt.keepfig = 2;
plt.style = ':';
plt = init_plot(plt);
for step = 1:size(seq.X,2)
    plt.vals = [sol(3),atan2(sol(4),sol(3))*180/pi,sol(5)];
    plt = update_plot(plt);
end
plt.style = '-';
xlabel(plt.axes(3),'time [s]')
ylabel(plt.axes(1),'m/s')
ylabel(plt.axes(2),'deg')
ylabel(plt.axes(3),'rad/s')
plt.m = [];
plt = init_plot(plt);
for i=1:size(seq.X,2)
    plt.vals = [seq.X(4,i),atan2(seq.X(5,i),seq.X(4,i))*180/pi,seq.X(6,i)];
    plt = update_plot(plt);
end