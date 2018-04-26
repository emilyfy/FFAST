function testObs_sim
filename = 'obs_sim.xlsx';
for rwd = 0:0.01:0.01
    for speed = 1:4
        sheetname = ['dr' num2str(rwd) 'sp' num2str(speed)];
        arr = [ {'max lateral deviation [m]'} {'min dist to obs [m]'} ...
                {'drift time [s]'} {'time taken [s]'} {'success'}];
        j = 1;
        unsucc = 0;
        while j <= 100
            [succ, maxdev, mindist, drift, time] = driftfun(1.5, speed, rwd);

            if succ==1
                arr(j+1,:) = [ {num2str(maxdev,'%.4f')} {num2str(mindist,'%.4f')} ...
                               {num2str(drift,'%.4f')} {num2str(time,'%.4f')} ...
                               {['=IF(B' num2str(j+1) '>Sheet1!K1,1,0)']} ];
                j = j+1;
            else
                unsucc = unsucc + 1;
            end
        end
        
        arr(102:103,:) = [ {'=AVERAGE(A2:A101)'} {'=AVERAGE(B2:B101)'} {'=AVERAGE(C2:C101)'} {'=AVERAGE(D2:D101)'} {'=AVERAGE(E2:E101)'} ;
                           {'=STDEV(A2:A101)'} {'=STDEV(B2:B101)'} {'=STDEV(C2:C101)'} {'=STDEV(D2:D101)'} {'=STDEV(E2:E101)'} ];
        xlswrite(filename,arr,sheetname,'A1:E103');
        
        arr = [ {'unsuccessful attempts'} {num2str(unsucc)} ];
        xlswrite(filename,arr,sheetname,'G1:H1');
        fprintf('%d m/s %f done\n', speed, rwd);
    end
end

end

function [succ, maxdev, mindist, drift, time] = driftfun(vsp,osp,rwd)

global vehicle
load('vehicle.mat')

% parameters
p.dt= 0.02;

p.m = vehicle.m;
p.b = vehicle.L_r;
p.a = vehicle.L_f;
p.G_f = vehicle.load_f;
p.G_r = vehicle.load_r;

p.c_x = vehicle.C_x;
p.c_a = vehicle.C_alpha;
p.Iz = vehicle.I_z;
p.mu = vehicle.mu;
p.mu_s = vehicle.mu_slide;

p.limThr = [-2 4];
p.limSteer = [-0.6  0.6];
p.xDes = [5 0 0 0 0 0];

p.cu = 1e-3*[1 0];
p.cdu = 1e-3*[1 6];

cf_ = [18 7 5 10 .1 .1];
p.cf = [cf_(1) 0 0 0 0 0];
p.pf= [ .01 .01 .1 .1 .1 .1];

p.cx  = 1e-1*[25 10 12];
p.px  = [.01 .01 .1];

p.Obs = [0 0];
p.d_thres = 0.3;
p.k_pos = 0.5;
p.k_vel = [0.4 0.0];

p.cdrift = -rwd;

p.lane_center = 0.0;
p.lane_thres = 0.20;
p.croad = 15;

Op.max_iter = 50;
T = 10;

if osp == 5
    x0 = [0;0;0;vsp;0;0;vsp;0;-1;0];
else
    x0 = [0;0;0;vsp;0;0;vsp;0;3;0];
end
u0(1,:) = 0.25*randn(1,T) + vsp;
u0(2,:) = 0.1*randn(1,T);

global debug
debug.rts = 0;

seq.rts = [0];

x = [x0(1:6);x0(9:10)];
dist2goal = p.xDes(1)-x(1);
step = 0;
set_obs = 0;

maxdev = 0;
mindist = 5;

while dist2goal > 0 && step<1000
    a = tic; [success, xx, uu, cost] = iLQGObs(x0, u0, p, Op); b = toc(a);
    
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
    
    x0 = [x(1:6) ; ulast(1) ; ulast(2) ; x(7:8) ];

end

time = step*p.dt;

if dist2goal < 0
    succ = 1;
else
    succ = 0;
end

drift = p.dt*sum(seq.rts);

end