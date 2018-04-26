clear all
%close all
clc

global vehicle
load('vehicle.mat')

p.m = vehicle.m;
p.L_f = vehicle.L_f;
p.L_r = vehicle.L_r;
p.load_f = vehicle.load_f;
p.load_r = vehicle.load_r;
p.I_z = vehicle.I_z;
p.c_x = vehicle.C_x;
p.c_a = vehicle.C_alpha;
p.mu = vehicle.mu;   
p.mu_s = vehicle.mu_slide;

p.limThr= [-3 6];
p.limSteer= [-0.6  0.6];

p.dt= 0.02;

%p.cu = 1e-2*[1 1];
%p.cdu = 1e-3*[4 6];
p.cu = [0 0];
p.cdu = 1e-3*[0 0];

p.cf = [10 10 10];
p.pf = [0 0 0];

Op.tolFun = 1e-20;
Op.lambdaMax = 1e20;
Op.max_iter = 5000;
T = 1;

c_dist = [.3;.3;.05;.07;.01;.03];
c_noise = [.3;.3;.05;.07;.01;.03;0;0];

vis.show_traj_cog = 1;
vis.show_wheels = 1;
vis.color = 'k';
vis.follow = 1;
vis.x_clearance = 2;
vis.y_clearance = 2;
plt.dt = 0.02;
plt.titles = {'v_x','v_y','yaw rate'};

plt = init_plot(plt);
vis = init_vis(vis);

%load('ss_cornering_val.mat');
%sol = ssval{3,:};
load('sol.mat');
sol = [sol(1);degtorad(5);1.5;sol(2);sol(3)];    %[rw;del;v_x;v_y;psi_dot]

p.goal = [sol(3);sol(4);sol(5)];
x0 = [0;0;0;sol(3);sol(4);sol(5);sol(1);sol(2)];
u0(1,:) = sol(1)*ones(1,T);
u0(2,:) = sol(2)*ones(1,T);
x = x0(1:6);

noise = 0;

j = T+1;
tic
pause(0.02)
for i=1:1000
    %[success, xx, uu, cost] = iLQGDriftCorneringSteadyState(x0, u0, p, Op);
    %fprintf('succ %d cost %f\n', success, cost)
    
    %dt = toc;tic
    %x = state_transition(x,[sol(1);sol(2)],p.dt);
    %x = state_transition(x,uu(:,1),p.dt) + noise*c_dist.*randn(size(x));
    
    %x0 = [x ; uu(1,1) ; uu(2,1)] + noise*c_noise.*randn(size(x0));
    %u0 = [uu(:,2:end) [sol(1);sol(2)] ];
    
    if j > T
        %u0 = [sol(1)+1e-2*randn(1,T);sol(2)+1e-2*randn(1,T)];
        u0 = [sol(1)*ones(1,T);sol(2)*ones(1,T)];
        [success, xx, uu, cost] = iLQGDriftCorneringSteadyState(x0, u0, p, Op);
        fprintf('succ %d cost %f\n', success, cost)
        j = 1;
    end
    
    x = state_transition(x,[sol(1);sol(2)],p.dt) + noise*c_dist.*randn(size(x));   % open-loop
    %x = state_transition(x,uu(:,j),p.dt) + noise*c_dist.*randn(size(x));           % closed-loop
    
    x0 = [x ; uu(1,j) ; uu(2,j)] + noise*c_noise.*randn(size(x0));
    j = j+1;
    
    vis = update_vis(vis,x,u0(:,1));
    plt.vals = [x(4),x(5),x(6)];
    plt = update_plot(plt);
end
