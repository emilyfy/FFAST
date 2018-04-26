function drift_sim
% Finds solution for cornering equilibrium states
% and simulates vehicle response under open-loop control

clear all
close all
clc

global vehicle
load('vehicle.mat')

global v_x
v_x = 1.5;
delta = degtorad(5); % set steering angle

Fyf = @(rw, v_y, psi_dot) tire_dyn_f(atan2((v_y+vehicle.L_f*psi_dot),curr_v_x())-delta);
Fyr = @(rw, v_y, psi_dot) tire_dyn_ry(curr_v_x(), rw, atan2((v_y-vehicle.L_r*psi_dot),curr_v_x()));
Fxr = @(rw, v_y, psi_dot) tire_dyn_rx(curr_v_x(), rw, atan2((v_y-vehicle.L_r*psi_dot),curr_v_x()));
F = @(V) [ (Fyf(V(1),V(2),V(3))*cos(delta)+Fyr(V(1),V(2),V(3)))/(vehicle.m*curr_v_x())-V(3); ...
           (vehicle.L_f*Fyf(V(1),V(2),V(3))*cos(delta)-vehicle.L_r*Fyr(V(1),V(2),V(3)))/vehicle.I_z; ...
           (Fxr(V(1),V(2),V(3))-Fyf(V(1),V(2),V(3))*sin(delta))/(vehicle.m*V(2))+V(3) ];

%InitialGuess = [1.5;0.05;0.3];    %stable cornering
InitialGuess = [3;1;-1.5];        %negative yaw rate
%InitialGuess = [3;-1;1.5];        %positive yaw rate

Options = optimoptions('fsolve'); 
Options.MaxIterations = 1000;
Options.MaxFunctionEvaluations = 5000;
Options.Display = 'iter';
Options.Algorithm = 'levenberg-marquardt';

% plot options
vis.show_traj_cog = 1;
vis.cog_color = 'g';
vis.show_traj_r = 1;
vis.rax_color = 'r';
vis.show_wheels = 1;
vis.color = 'k';
vis.follow = 1;
vis.x_clearance = 2;
vis.y_clearance = 2;
plt.dt = 0.02;
plt.titles = {'v_x','\beta','yaw rate'};

plt = init_plot(plt);
vis = init_vis(vis);

xlabel(plt.axes(3),'time [s]')
ylabel(plt.axes(1),'m/s')
ylabel(plt.axes(2),'deg')
ylabel(plt.axes(3),'rad/s')

sol = fsolve(F, InitialGuess, Options);
fprintf('sol: rw = %f v_y = %f yaw_rate = %f\n', sol(1), sol(2), sol(3))
sb0 = F(sol);
fprintf('shouldbezero: [%f %f %f]\n', sb0(1),sb0(2),sb0(3))
save('sol.mat','sol')
pause(1)

Options.MaxIterations = 5;
Options.MaxFunctionEvaluations = 10;
Options.Display = 'off';

%X = [0;0;0;0;0;0];
X = [0;0;0;v_x;sol(2);sol(3)];

c_noise = [.3;.3;.05;.07;.01;.03];

global debug
debug.beta = 0;
debug.rts = 0;
debug.cs = 0;

tic
dt = 0.02;

while 1
    U = [sol(1); delta]; 
    
    %dt = toc; tic
    X = state_transition(X,U,dt);% + 1e-2*c_noise.*randn(size(X));
    
    vis = update_vis(vis, X, U);
    
    v_x = X(4);
    v_y = X(5);
    psi_dot = X(6);
    %sol = fsolve(F, sol, Options);
    sb0 = F([sol(1);v_y;psi_dot]);
    %fprintf('shouldbezero: [%f %f %f]\n', sb0(1),sb0(2),sb0(3))
    
    %fprintf('rw = %f v_x = %f v_y = %f\n', sol(1), v_y, psi_dot)
    %fprintf('rts = %d beta = %f cs = %d\n', debug.rts, radtodeg(debug.beta), debug.cs)
    
    plt.vals = [X(4),atan2(X(5),X(4))*180/pi,X(6)];
    plt = update_plot(plt);
end

end

function ret = tire_dyn_ry(v_x, wheel_v, alpha)
    [~, ret] = tire_dyn_r(v_x, wheel_v, alpha);
end
function ret = tire_dyn_rx(v_x, wheel_v, alpha)
    [ret, ~] = tire_dyn_r(v_x, wheel_v, alpha);
end
function ret = curr_v_x()
    global v_x
    ret = v_x;
end
function ret = curr_v_y()
    global v_y
    ret = v_y;
end
function ret = curr_psi_dot()
    global psi_dot
    ret = psi_dot;
end
