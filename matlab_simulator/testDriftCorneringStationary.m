clear all
close all
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

p.cu = 1e-3*[1 1];
p.cdu = 1e-4*[1 6];

p.cx = [1 1 0 5 5 5];
p.px = [.01 .01 0 .1 .1 .1];

p.cf = [10 10 10];
p.pf = [0.1 0.1 0.1];

Op.tolFun = 1e-20;
Op.lambdaMax = 1e20;
Op.max_iter = 5000;
T = 50;

vis.show_traj_cog = 1;
vis.cog_color = 'g';
vis.show_traj_r = 0;
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

load('ss_cornering_val.mat');
sol = ssval{3,:};
%load('sol.mat');
%sol = [sol(1);degtorad(5);1.5;sol(2);sol(3)];    %[rw;del;v_x;v_y;psi_dot]

p.goal = [sol(3);sol(4);sol(5)];
x0 = [0;0;0;0;0;0;0.5*randn+sol(1);0.3*randn];
u0(1,:) = 0.5*randn(1,T) + sol(1);
u0(2,:) = 0.3*randn(1,T);
x = x0(1:6);

for i=1:10
    x = state_transition(x,[sol(1);sol(2)],p.dt);
    vis = update_vis(vis,x,[sol(1);sol(2)]);
    plt.vals = [x(4),x(5),x(6)];
    plt = update_plot(plt);
end

while sum(abs(x(4:6) - p.goal)) > 0.2
    a = tic; [success, xx, uu, cost] = iLQGDriftCorneringStationary(x0, u0, p, Op); b = toc(a);
    
    if b <= T*p.dt
        for i=1:ceil(b/p.dt)
            x = state_transition(x,uu(:,i),p.dt);
            vis = update_vis(vis,x,uu(:,i));
            plt.vals = [x(4),x(5),x(6)];
            plt = update_plot(plt);
        end
        ulast = uu(:,i);
        u0 = [uu(:,i+1:end) [0.5*randn(1,i)+sol(1);0.3*randn(1,i)] ];
    else
        for i=1:T
            x = state_transition(x,uu(:,i),p.dt);
            vis = update_vis(vis,x,uu(:,i));
            plt.vals = [x(4),x(5),x(6)];
            plt = update_plot(plt);
        end
        for i=T+1:ceil(b/p.dt)
            x = state_transition(x,uu(:,T),p.dt);
            vis = update_vis(vis,x,uu(:,T));
            plt.vals = [x(4),x(5),x(6)];
            plt = update_plot(plt);
        end
        ulast = uu(:,T);
        u0 = [0.5*randn(1,T)+sol(1);0.3*randn(1,T)];
    end
    
    x0 = [x(1:6) ; ulast(1) ; ulast(2) ];
    
    %for i=1:T
    %    [success, xx, uu, cost] = iLQGDriftCorneringStationary(x0, u0, p, Op); %mpc?
    %    fprintf('succ %d cost %f\n', success, cost)

    %    x = state_transition(x,uu(:,1),p.dt);

    %    x0 = [x ; uu(1,1) ; uu(2,1)];
    %    u0 = [uu(:,2:end) [0.5*randn+sol(1);0.3*randn] ];

    %    vis = update_vis(vis,x,uu(:,1));
    %    plt.vals = [x(4),x(5),x(6)];
    %    plt = update_plot(plt);
    %end
end
