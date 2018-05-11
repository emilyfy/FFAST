clear all
load('../mats/1.mat')

global vehicle
load('vehicle.mat')

seq.X = [];
seq.U = [];

last_cmd_idx = 2;
start_time = odom{1,'rosbag_recv_time'};
times = [];
for odom_idx = 1:size(odom,1)-10
    curr_time = odom{odom_idx,'rosbag_recv_time'} - start_time;
    times = [times curr_time];
    
    for i = last_cmd_idx:size(cmd,1)
        cmd_time = cmd{i,'rosbag_recv_time'} - start_time;
        if cmd_time > curr_time
            last_cmd_idx = i;
            break
        end
    end
    cmd_idx = last_cmd_idx-1;
    
    pos_x = odom{odom_idx,'pose_pose_position_x'};
    pos_y = odom{odom_idx,'pose_pose_position_y'};
    eul = quat2eul([odom{odom_idx,'pose_pose_orientation_w'},odom{odom_idx,'pose_pose_orientation_x'},odom{odom_idx,'pose_pose_orientation_y'},odom{odom_idx,'pose_pose_orientation_z'}]);
    yaw = eul(1);
    
    v_x = odom{odom_idx,'twist_twist_linear_x'};
    v_y = odom{odom_idx,'twist_twist_linear_y'};
    yaw_rate = odom{odom_idx,'twist_twist_angular_z'};
    
    cmd_sp = cmd{cmd_idx,'drive_speed'};
    cmd_steer = cmd{cmd_idx,'drive_steering_angle'};
    
    seq.X = [seq.X [pos_x;pos_y;yaw;v_x;v_y;yaw_rate] ];
    seq.U = [seq.U [cmd_sp;cmd_steer] ];
end

dt = [];
for i=2:size(times,2)
    dt = [dt times(i)-times(i-1)];
end
dt = mean(dt);

close all
plt.titles = {'v_x','\beta','yaw rate'};
plt.keepfig = 1;
plt.dt = dt;
plt = init_plot(plt);

rvis.show_traj_cog = 1;
rvis.show_wheels = 1;
rvis.keepfig = 2;
rvis.follow = 1;
rvis.x_clearance = 2;
rvis.y_clearance = 2;
rvis = init_vis(rvis);
mvis.show_traj_cog = 1;
mvis.cog_color = 'r';
mvis.keepfig = 2;
mvis = init_vis(mvis);
x = seq.X(:,1);
u = seq.U(:,1);

for i=1:size(seq.X,2)
    rvis = update_vis(rvis,seq.X(:,i),seq.U(:,i));
    mvis = update_vis(mvis,x,u);
    x = state_transition(x,u,times(i+1)-times(i));
    u = seq.U(:,i+1);
    plt.vals = [seq.X(4,i),atan2(seq.X(5,i),seq.X(4,i))*180/pi,seq.X(6,i)];
    plt = update_plot(plt);
end