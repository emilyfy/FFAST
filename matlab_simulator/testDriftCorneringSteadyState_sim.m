function testDriftSteadyState_sim

for cs = 0:1
    
    for i=1:7
        close all
        clc
        drift_fun(cs,i);
        %diff = drift_fun(cs,i);
        %data(:,:,i) = diff;
    end
    
    % save integral of differences to excel
    %{
    fprintf('all figures saved.\n')
    fprintf('saving to excel...\n')
    
    filename = 'drift_sim_1_5mps.xlsx';
    vars = [{'vx'} {'vy'} {'yawrate'}];
    for i=1:3
        
        if cs==0
            sheetname = ['ncs ' vars{i}];
        elseif cs==1
            sheetname = ['cs ' vars{i}];
        end
        
        arr = [ {'steering angle [deg]'} {'open-loop'} ...
                {'closed-loop no noise'} {'closed-loop with noise'} ];

        sa = 0:5:30;
        for j=1:7
            
            c = cell(1,4);
            c{1} = num2str(sa(j));
            for k = 1:3
                c{k+1} = num2str(data(i,k,j));
            end
            
            arr(j+1,:) = c;
        end
        
        xlswrite(filename,arr,sheetname,'A1:D8');
    end
    fprintf('done.\n')
    %}
end

end

function diff = drift_fun(cs,saidx)

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

p.cu = [0 0];
p.cdu = [0 0];

p.cf = [10 10 10];
p.pf = [0 0 0];

Op.tolFun = 1e-20;
Op.lambdaMax = 1e20;
Op.max_iter = 5000;
T = 1;

c_dist = [.3;.3;.05;.07;.01;.03];
c_noise = [.3;.3;.05;.07;.01;.03;0;0];

load('ss_cornering_val.mat');
if cs == 0 %non countersteering
    idx = 2:2:14;
    csstr = 'ncs';
elseif cs == 1 %countersteering
    idx = 1:2:13;
    csstr = 'cs';
end

sas = 0:5:30;
sa = sas(saidx);
sol = ssval{idx(saidx),:};

p.xDes = [sol(3);sol(4);sol(5)];

stl = [{'-'} {'--'} {'-.'}];
plt.dt = 0.02;
plt.titles = {'v_x','v_y','yaw rate'};
plt.keepfig = 1;

% -plot reference traj-
    plt.style = ':';
    plt = init_plot(plt);
    for step = 1:1000
        plt.vals = [sol(3),sol(4),sol(5)];
        plt = update_plot(plt);
    end
% -plot reference traj-

for cl = 0:2  % open-loop, closed-loop no noise, closed-loop with 0.1 noise
    plt.style = stl{cl+1};
    plt = init_graph(plt);

    if cl == 2
        noise = 1e-1;
    else
        noise = 0;
    end
    
    diff(:,cl+1) = [0;0;0];
    
    x0 = [0;0;0;sol(3);sol(4);sol(5);sol(1);sol(2)];
    u0(1,:) = sol(1)*ones(1,T);
    u0(2,:) = sol(2)*ones(1,T);
    x = x0(1:6);
    
    j = T+1;
    for step = 1:1000
        if cl == 0
            x = state_transition(x,[sol(1);sol(2)],p.dt);
        else
            if j > T
                %u0 = [sol(1)+1e-2*randn(1,T);sol(2)+1e-2*randn(1,T)];
                u0 = [sol(1)*ones(1,T);sol(2)*ones(1,T)];
                [success, xx, uu, cost] = iLQGDriftCorneringSteadyState(x0, u0, p, Op);
                j = 1;
            end
            x = state_transition(x,uu(:,j),p.dt) + noise*c_dist.*randn(size(x));
            x0 = [x ; uu(1,j) ; uu(2,j)] + noise*c_noise.*randn(size(x0));
        end
        j = j+1;
        
        plt.vals = [x(4),x(5),x(6)];
        plt = update_plot(plt);

        %diff(:,cl+1) = diff(:,cl+1) + abs([x(4);x(5);x(6)]-p.xDes)*p.dt;
    end
end

axis(plt.axes(1),[-0.5, 21, -inf, inf]);
axis(plt.axes(2),[-0.5, 21, -inf, inf]);
axis(plt.axes(3),[-0.5, 21, -inf, inf]);
ylabel(plt.axes(1),'m/s');
ylabel(plt.axes(2),'m/s');
ylabel(plt.axes(3),'rad/s');
xlabel(plt.axes(3),'time [s]');
%plt.lgd = legend(plt.ax(1),'open-loop','closed-loop no noise','closed-loop with noise');

filename = ['drift_sim_graphs/' csstr num2str(sa) '.bmp'];
saveas(plt.fig,filename,'bmp');
%filename = ['drift_sim_graphs/' csstr num2str(sa) '1.bmp'];
%saveas(plt.fig,filename,'bmp');
%plt.lgd.Location = 'northwest';
%filename = ['drift_sim_graphs/' csstr num2str(sa) '2.bmp'];
%saveas(plt.fig,filename,'bmp');

end
