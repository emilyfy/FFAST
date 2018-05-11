function testDriftCornering_sim

times = [];
dc_time = [];

for cs = 0:1
    for saidx=1:7
        for i=1:100
            times(cs+1,saidx,i) = drift_fun(cs,saidx);
        end
        dc_time(cs+1,saidx) = mean(times(cs+1,saidx,:));
    end
end
save('driftcornering_time.mat','dc_time')
end

function t = drift_fun(cs,saidx)

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

c_dist = [.3;.3;.05;.07;.01;.03];
c_noise = [.3;.3;.05;.07;.01;.03;0;0];

load('ss_cornering_val.mat');
if cs == 0 %non countersteering
    idx = 2:2:14;
elseif cs == 1 %countersteering
    idx = 1:2:13;
end

sas = 0:5:30;
sa = sas(saidx);
sol = ssval{idx(saidx),:};

p.goal = [sol(3);sol(4);sol(5)];

x0 = [0;0;0;0;0;0;0.2*randn+0.8*sol(1);0.1*randn+0.07*sol(5)];
u0(1,:) = 0.2*randn(1,T) + 0.8*sol(1);
u0(2,:) = 0.1*randn(1,T) + 0.07*sol(5);
x = x0(1:6);

X = [];
U = [];
n = 0;

for i=1:10
    x = state_transition(x,[0.8*sol(1);0.07*sol(5)],p.dt);
    X = [X x];
    U = [U [0.8*sol(1);0.07*sol(5)]];
    n = n+1;
end

while abs(x(4)-sol(3)) > 0.1 && abs(atan2(x(5),x(4))-atan2(sol(5),sol(4))) > 0.1 && abs(x(6)-sol(5)) > 0.2
    a = tic; [success, xx, uu, cost] = iLQGDriftCorneringStationary(x0, u0, p, Op); b = toc(a);

    if b <= T*p.dt
        for i=1:ceil(b/p.dt)
            x = state_transition(x,uu(:,i),p.dt);
            X = [X x];
            U = [U uu(:,i)];
            n = n+1;
        end
        ulast = uu(:,i);
        u0 = [uu(:,i+1:end) [0.2*randn(1,i)+0.8*sol(1);0.1*randn(1,i)+0.07*sol(5)] ];
    else
        for i=1:T
            x = state_transition(x,uu(:,i),p.dt);
            X = [X x];
            U = [U uu(:,i)];
            n = n+1;
        end
        for i=T+1:ceil(b/p.dt)
            x = state_transition(x,uu(:,T),p.dt);
            X = [X x];
            U = [U uu(:,T)];
            n = n+1;
        end
        ulast = uu(:,T);
        u0 = [0.2*randn(1,T)+0.8*sol(1);0.1*randn(1,T)+0.07*sol(5)];
    end

    x0 = [x(1:6) ; ulast(1) ; ulast(2) ];
end

t = p.dt*n;

end