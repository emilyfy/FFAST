function testObs_ros_sim
    arr = [];
    for vsp = 1:4
        for osp = 1:5
            arr(vsp,osp).unsucc_num = 0;
            arr(vsp,osp).sim = table();
            
            i = 0;
            while i < 100
                [tsucc, maxdev, mindist, drift, time, osucc] = driftfun(vsp,osp);
                if tsucc==1
                    i = i+1;
                    arr(vsp,osp).sim{i,:} = [maxdev mindist drift time osucc];
                elseif tsucc==0
                    arr(vsp,osp).unsucc_num = arr(vsp,osp).unsucc_num + 1;
                end
            end
        
            arr(vsp,osp).sim.Properties.VariableNames = ...
                [ {'max_dev'} {'min_dist'} {'drift_time'} {'time'} {'succ'}];
            
            arr(vsp,osp).maxdev = mean(arr(vsp,osp).sim{:,1});
            arr(vsp,osp).mindist = mean(arr(vsp,osp).sim{:,2});
            arr(vsp,osp).dtime = mean(arr(vsp,osp).sim{:,3});
            arr(vsp,osp).time = mean(arr(vsp,osp).sim{:,4});
            arr(vsp,osp).succ = mean(arr(vsp,osp).sim{:,5});
            
            fprintf('veh %d m/s obs %d m/s done\n', vsp,osp);
        end
    end
    save('obs_sim_results.mat','arr');
end

function [tsucc, maxdev, mindist, drift, time, osucc] = driftfun(vsp,osp)

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

coll_thr = 0.5*vehicle.L + 0.5*vehicle.tw + 0.05;

p.limThr = [-2 4];
p.limSteer = [-0.6  0.6];
p.xDes = [5 0 0 0 0 0];

p.cu = 1e-3*[1 0];
p.cdu = 1e-3*[1 6];

cf_ = [18 7 5 1 .1 .1];
p.cf = [cf_(1) 0 0 0 0 0];
p.pf= [ .01 .01 .1 .1 .1 .1];

p.cx  = 1e-1*[25 10 12];
p.px  = [.01 .01 .1];

p.Obs = [0 0];
p.d_thres = 0.6;
p.k_pos = 15;
p.k_vel = [0.1 0.01];

p.lane_center = 0.0;
p.lane_thres = 0.20;
p.croad = 7;

Op.max_iter = 50;
T = 10;

if osp == 5
    x0 = [0;0;0;vsp;0;0;vsp;0;-1;0];
else
    x0 = [0;0;0;vsp;0;0;vsp;0;3;0];
end
u0(1,:) = 0.25*randn(1,T) + vsp;
u0(2,:) = 0.1*randn(1,T);

% visualize
%{
vis.show_traj_cog = 1;
vis.show_traj_r = 0;
vis.show_wheels = 1;
close all
vis = init_vis(vis);
%}

global debug
debug.rts = 0;

seq.rts = [0];

x = [x0(1:6);x0(9:10)];
dist2goal = p.xDes(1)-x(1);
step = 0;
set_obs = 0;

maxdev = 0;
mindist = 5;
osucc = 1;

while dist2goal > 0 && step<500
    a = tic; [success, xx, uu, cost] = iLQGObs_ros(x0, u0, p, Op); b = toc(a);
    
    if b <= T*p.dt
        for i=1:ceil(b/p.dt)
            x = state_transition(x,uu(:,i),p.dt,p);
            updatevars;
        end
        ulast = uu(:,i);
        u0 = [uu(:,i+1:end) [0.5*randn(1,i)+vsp ; 0.1*randn(1,i)] ];
    else
        for i=1:10
            x = state_transition(x,uu(:,i),p.dt,p);
            updatevars;
        end
        for i=11:ceil(b/p.dt)
            x = state_transition(x,[0;0],p.dt,p);
            updatevars;
        end
        ulast = [0;0];
        u0 = [0.25*randn(1,T)+vsp ; 0.1*randn(1,T)];
    end
    
    x0 = [x(1:6) ; ulast(1) ; ulast(2) ; x(7:8)];
    
end

time = step*p.dt;

if dist2goal <= 0.15
    tsucc = 1;
else
    tsucc = 0;
end

drift = p.dt*sum(seq.rts);

end