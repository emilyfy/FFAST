clear all
load('../mats/d1.mat')

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

p.limThr = [-2 4];
p.limSteer = [-0.6  0.6];
p.goal = [5 0 0 0 0 0];

p.cu = 1e-3*[1 0];
p.cdu = 1e-3*[1 6];

cf_ = [18 7 5 10 .1 .1];
p.cf = [cf_(1) 0 0 0 0 0];
p.pf = [ .01 .01 .1 .1 .1 .1];

p.cx  = 1e-1*[25 10 12];
p.px  = [.01 .01 .1];

p.obs_vel = [0 0];
p.dist_obs_thres = 0.3;
p.c_obs = [0.5 0.4 0.0];

p.c_drift = -0.00;

p.lane_center = 0.0;
p.lane_thres = 0.20;
p.c_lane = 15;

Op.max_iter = 50;
T = 10;

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
seq.obs = [];

%% calculate

last_cmd_idx = 2;
last_obs_idx = 2;
last_input_idx = 2;
last_output_idx = 2;

start_time = timer{1,'rosbag_recv_time'};
end_time = timer{end,'rosbag_recv_time'} - start_time;

% find starting odom msg
for i = 1:size(odom,1)
    odom_time = odom{i,'rosbag_recv_time'} - start_time;
    if odom_time > 0
        starting_odom_idx = i;
        break
    end
end

times = [];
for odom_idx = starting_odom_idx:size(odom,1)
    curr_time = odom{odom_idx,'rosbag_recv_time'} - start_time;
    times = [times curr_time];
    
    % make sure it's not beyond end time
    if curr_time > end_time
        ending_odom_idx = odom_idx;
        break
    end
    
    % find corresponding messages
    for i = last_cmd_idx:size(cmd,1)
        cmd_time = cmd{i,'rosbag_recv_time'} - start_time;
        if cmd_time > curr_time
            last_cmd_idx = i;
            break
        end
    end
    cmd_idx = last_cmd_idx-1;
    
    for i = last_obs_idx:size(obs,1)
        obs_time = obs{i,'rosbag_recv_time'} - start_time;
        if obs_time > curr_time
            last_obs_idx = i;
            break
        end
    end
    obs_idx = last_obs_idx-1;
    
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
    
    obs_x = obs{obs_idx,'circles'}{1}.center.x;
    obs_y = obs{obs_idx,'circles'}{1}.center.y;
    obs_vel_x = obs{obs_idx,'circles'}{1}.velocity.x;
    obs_vel_y = obs{obs_idx,'circles'}{1}.velocity.x;
    
    prev_cmd_sp = ilqr_input{input_idx,'previouscommand_speed'};
    prev_cmd_steer = ilqr_input{input_idx,'previouscommand_steering_angle'};
    
    % ilqr in matlab
    x0 = [pos_x;pos_y;yaw;
          v_x;v_y;yaw_rate;
          prev_cmd_sp;prev_cmd_steer;
          obs_x;obs_y];
    
    u0 = [];
    n = size(ilqr_input{input_idx,'remainingcommands'}{1},2);
    for i=1:n
        u0 = [u0 [ilqr_input{input_idx,'remainingcommands'}{1}{i}.speed; ilqr_input{input_idx,'remainingcommands'}{1}{i}.steering_angle] ];
    end
    u0 = [u0 [0.25*randn(1,10-n)+1.5 ; 0.1*randn(1,10-n)] ];

    p.obs_vel = [obs_vel_x obs_vel_y];
    
    if p.goal(1)-pos_x < 0.1
        p.cf = [0 cf_(2:6)];
    end
    
    [success, xx, uu, cost] = iLQGObs_ros(x0, u0, p, Op);

    % comparing matlab output and ros msg output
    seq.succ_mat = [seq.succ_mat success];
    seq.path_mat = cat(3,seq.path_mat,xx);
    seq.U_mat = cat(3,seq.U_mat,uu);
    seq.cost_mat = [seq.cost_mat cost];

    for i=1:11
        xx_ros(1,i) = ilqr_output{output_idx,'states'}{i}.pose.x;
        xx_ros(2,i) = ilqr_output{output_idx,'states'}{i}.pose.y;
        xx_ros(3,i) = ilqr_output{output_idx,'states'}{i}.pose.theta;
        xx_ros(4,i) = ilqr_output{output_idx,'states'}{i}.twist.x;
        xx_ros(5,i) = ilqr_output{output_idx,'states'}{i}.twist.y;
        xx_ros(6,i) = ilqr_output{output_idx,'states'}{i}.twist.theta;
    end
    for i=1:10
        uu_ros(1,i) = ilqr_output{output_idx,'commands'}{i}.speed;
        uu_ros(2,i) = ilqr_output{output_idx,'commands'}{i}.steering_angle;
    end
    seq.succ_ros = [seq.succ_ros ilqr_output{output_idx,'success'}];
    seq.path_ros = cat(3,seq.path_ros,xx_ros);
    seq.U_ros = cat(3,seq.U_ros,uu_ros);
    seq.cost_ros = [seq.cost_ros ilqr_output{output_idx,'cost'}];
    
    seq.X = [seq.X [pos_x;pos_y;yaw;v_x;v_y;yaw_rate] ];
    seq.U = [seq.U [prev_cmd_sp;prev_cmd_steer] ];
    seq.obs = [seq.obs [obs_x;obs_y] ];
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
rvis.axlims = [-vehicle.L_r,5+vehicle.L_f,-2.5,2.5];
rvis = init_vis(rvis);
mvis.show_wheels = 1;
mvis.path_color = 'b';
mvis.keepfig = 1;
mvis.axlims = [-vehicle.L_r,5+vehicle.L_f,-2.5,2.5];
mvis = init_vis(mvis);

%mvis.m = [];
for i=1:size(seq.X,2)
    %mvis.txt = text(1.7,-2.3,['time =' num2str(times(:,i)) ' s']);
    mvis = update_vis(mvis, seq.X(:,i), seq.U(:,i), seq.obs(:,i), seq.path_mat(:,:,i));
    rvis = update_vis(rvis, seq.X(:,i), seq.U(:,i), seq.obs(:,i), seq.path_ros(:,:,i));
    %delete(mvis.txt)
end
%mov = mvis.m;
%save(['../movs/' num2str(exp_no) '.mat'],'mov','dt')