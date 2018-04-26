clear all
close all
clc

global vehicle
load('vehicle.mat')

% parameters
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
p.pf= [ .01 .01 .1 .1 .1 .1];

p.cx  = 1e-1*[25 10 12];
p.px  = [.01 .01 .1];

p.obs_vel = [0 0];
p.dist_obs_thres = 0.6;
p.c_obs = [15 0.1 0.01];

p.lane_center = 0.0;
p.lane_thres = 0.20;
p.c_lane = 7;

Op.max_iter = 50;
T = 10;

x0 = [0;0;0;1.5;0;0;1.5;0;3;0];
u0(1,:) = 0.25*randn(1,T) + 1.5;
u0(2,:) = 0.1*randn(1,T);

vis.show_traj_cog = 1;
vis.show_wheels = 1;
vis.show_grid = 1;
vis.axlims = [-vehicle.L_r,5+vehicle.L_f,-2.5,2.5];
vis.lane_lines = [-1,6,-0.5,-0.5;-1,6,0.5,0.5];
vis = init_vis(vis);

global debug
debug.rts = 0;

seq.X = [x0(1:6)];
seq.U = [];
seq.obs = [x0(9:10)];
seq.path = [];
seq.rts = [0];

x = [x0(1:6);x0(9:10)];
dist2goal = p.goal(1)-x(1);
step = 0;
set_obs = 0;
vsp = 1.5;
osp = 5;
if osp == 5
    x(7) = -1;
end

while dist2goal > 0 && step<1000
    a = tic;[success, xx, uu, cost] = iLQGObs_ros(x0, u0, p, Op);b = toc(a);
    
    if b <= T*p.dt
        for i=1:ceil(b/p.dt)
            x = state_transition(x,uu(:,i),p.dt,p);
            updatevars;
        end
        ulast = uu(:,i);
        u0 = [uu(:,i+1:end) [0.25*randn(1,i)+1.5 ; 0.1*randn(1,i)] ];
    else
        for i=1:T
            x = state_transition(x,uu(:,i),p.dt,p);
            updatevars;
        end
        for i=T+1:ceil(b/p.dt)
            x = state_transition(x,[0;0],p.dt,p);
            updatevars;
        end
        ulast = [0;0];
        u0 = [0.25*randn(1,T)+1.5 ; 0.1*randn(1,T)];
    end
    
    x0 = [x(1:6) ; ulast(1) ; ulast(2) ; x(7:8)];
    
end

vis = vis_seq(vis,seq);
