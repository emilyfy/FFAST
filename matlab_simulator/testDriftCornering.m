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

p.limThr = [-3 6];
p.limSteer = [-0.6  0.6];

p.dt = 0.02;

p.cu = 1e-3*[1 1];
p.cdu = 1e-4*[1 6];

p.cx = [1 1 0 5 5 5];
p.px = [.01 .01 0 .1 .1 .1];

p.cf = [10 10 10];
p.pf = [0.1 0.1 0.1];

Op.tolFun = 1e-20;
Op.lambdaMax = 1e20;
Op.max_iter = 500;
T = 50;

vis.show_traj_cog = 1;
vis.cog_color = 'g';
vis.show_wheels = 1;
vis.color = 'k';
vis.follow = 1;
vis.x_clearance = 2;
vis.y_clearance = 2;
plt.titles = {'v_x','\beta','yaw rate'};
plt.dt = 0.02;

plt = init_plot(plt);
vis = init_vis(vis);

xlabel(plt.axes(3),'time [s]')
ylabel(plt.axes(1),'m/s')
ylabel(plt.axes(2),'deg')
ylabel(plt.axes(3),'rad/s')

load('ss_cornering_val.mat');
sol = ssval{3,:};
%load('sol.mat');
%sol = [sol(1);degtorad(5);1.5;sol(2);sol(3)];    %[rw;del;v_x;v_y;psi_dot]

p.goal = [sol(3);sol(4);sol(5)];
x0 = [0;0;0;0;0;0;0.2*randn+0.8*sol(1);0.1*randn+0.07*sol(5)];
u0(1,:) = 0.2*randn(1,T) + 0.8*sol(1);
u0(2,:) = 0.1*randn(1,T) + 0.07*sol(5);
x = x0(1:6);

X = [];
U = [];
plts = [];
n = 0;

for i=1:10
    x = state_transition(x,[sol(1);sol(2)],p.dt);
    X = [X x];
    U = [U [0.8*sol(1);0.07*sol(5)]];
    plts = [plts; x(4),atan2(x(5),x(4))*180/pi,x(6)];
    n = n+1;
end

while sum(abs(x(4:6) - p.goal)) > 0.2
    a = tic; [success, xx, uu, cost] = iLQGDriftCorneringStationary(x0, u0, p, Op); b = toc(a);
    
    if b <= T*p.dt
        for i=1:ceil(b/p.dt)
            x = state_transition(x,uu(:,i),p.dt);
            X = [X x];
            U = [U uu(:,i)];
            plts = [plts; x(4),atan2(x(5),x(4))*180/pi,x(6)];
            n = n+1;
        end
        ulast = uu(:,i);
        u0 = [uu(:,i+1:end) [0.2*randn(1,i)+0.8*sol(1);0.1*randn(1,i)+0.07*sol(5)] ];
    else
        for i=1:T
            x = state_transition(x,uu(:,i),p.dt);
            X = [X x];
            U = [U uu(:,i)];
            plts = [plts; x(4),atan2(x(5),x(4))*180/pi,x(6)];
            n = n+1;
        end
        for i=T+1:ceil(b/p.dt)
            x = state_transition(x,[0.8*sol(1);0.07*sol(5)],p.dt);
            X = [X x];
            U = [U uu(:,T)];
            plts = [plts; x(4),atan2(x(5),x(4))*180/pi,x(6)];
            n = n+1;
        end
        ulast = uu(:,T);
        u0 = [0.2*randn(1,T)+0.8*sol(1);0.1*randn(1,T)+0.07*sol(5)];
    end
    
    x0 = [x(1:6) ; ulast(1) ; ulast(2) ];
    
end

for i = 1:n
    vis = update_vis(vis,X(:,i),U(:,i));
    plt.vals = plts(i,:);
    plt = update_plot(plt);
    pause(0.02)
end

fprintf('Starting steady state.\n');

p.cu = [0 0];
p.cdu = [0 0];
T = 1;
j = T+1;
for i=1:250
    if j > T
        u0 = [sol(1)*ones(1,T);sol(2)*ones(1,T)];
        [success, xx, uu, cost] = iLQGDriftCorneringSteadyState(x0, u0, p, Op);
        j = 1;
    end
    
    x = state_transition(x,uu(:,j),p.dt);
    
    x0 = [x ; uu(1,j) ; uu(2,j)];
    j = j+1;
    
    vis = update_vis(vis,x,u0(:,1));
    plt.vals = [x(4),atan2(x(5),x(4))*180/pi,x(6)];
    plt = update_plot(plt);
end