function testiLQGDriftSteadyState

clear all
close all
clc

global vehicle
load('vehicle.mat')

load('ss_cornering_val.mat');
global sol
sol = ssval{3,:};

full_DDP = false;

DYNCST  = @(x,u,i) car_dyn_cst(x,u,full_DDP);
T       = 5;
x       = [0;0;0;sol(3);sol(4);sol(5)];
u       = [sol(1);sol(2)]*ones(1,T);
Op.lims = [ -3  6;
           -.6 .6];
Op.maxIter = 5000;

vis.show_traj_cog = 1;
vis.cog_color = 'g';
vis.show_traj_r = 1;
vis.rax_color = 'r';
vis.show_wheels = 1;
vis.color = 'k';
vis.follow = 1;
vis.x_clearance = 2;
vis.y_clearance = 2;

X = [x];
U = [];

i = T+1;
for j = 1:500
    if i > T
        [xx,uu]= iLQG(DYNCST, x, u, Op);
        i = 1;
    end
    
    x = state_transition(x,uu(:,i),0.02);
    
    X = [X x];
    U = [U uu(:,i)];
    i = i+1;
end

U(:,size(X,2))=[0;0];
vis = init_vis(vis);
pause()
for j = 1:size(X,2)
    vis = update_vis(vis,X(:,j),U(:,j));
    pause(0.02)
end

end

function y = car_dynamics(x,u)
dt = 0.02;

dx = zeros(size(x));
for i=1:size(x,2)
    for j=1:size(x,3)
        dx(:,i,j) = dynamics(x(:,i,j),u(:,i,j));
    end
end
y  = x + dx.*dt;
end

function c = car_cost(x, u)
% cost function

global sol
dx = pp(x(4:6,:),[-sol(3);-sol(4);-sol(5)]);

cx  = [.1 .1 .1];
px  = [.001 .001 .001]';

lx = cx*sabs(dx,px);

% total cost
c     = lx;
end

function y = sabs(x,p)
% smooth absolute-value function (a.k.a pseudo-Huber)
y = pp( sqrt(pp(x.^2,p.^2)), -p);
end

function [f,c,fx,fu,fxx,fxu,fuu,cx,cu,cxx,cxu,cuu] = car_dyn_cst(x,u,full_DDP)
% combine car dynamics and cost
% use helper function finite_difference() to compute derivatives

if nargout == 2
    f = car_dynamics(x,u);
    c = car_cost(x,u);
else
    % state and control indices
    ix = 1:6;
    iu = 7:8;
    
    % dynamics first derivatives
    xu_dyn  = @(xu) car_dynamics(xu(ix,:),xu(iu,:));
    J       = finite_difference(xu_dyn, [x; u]);
    fx      = J(:,ix,:);
    fu      = J(:,iu,:);
    
    % dynamics second derivatives
    if full_DDP
        xu_Jcst = @(xu) finite_difference(xu_dyn, xu);
        JJ      = finite_difference(xu_Jcst, [x; u]);
        JJ      = reshape(JJ, [6 8 size(J)]);
        JJ      = 0.5*(JJ + permute(JJ,[1 3 2 4])); %symmetrize
        fxx     = JJ(:,ix,ix,:);
        fxu     = JJ(:,ix,iu,:);
        fuu     = JJ(:,iu,iu,:);    
    else
        [fxx,fxu,fuu] = deal([]);
    end    
    
    % cost first derivatives
    xu_cost = @(xu) car_cost(xu(ix,:),xu(iu,:));
    J       = squeeze(finite_difference(xu_cost, [x; u]));
    cx      = J(ix,:);
    cu      = J(iu,:);
    
    % cost second derivatives
    xu_Jcst = @(xu) squeeze(finite_difference(xu_cost, xu));
    JJ      = finite_difference(xu_Jcst, [x; u]);
    JJ      = 0.5*(JJ + permute(JJ,[2 1 3])); %symmetrize
    cxx     = JJ(ix,ix,:);
    cxu     = JJ(ix,iu,:);
    cuu     = JJ(iu,iu,:);
    
    [f,c] = deal([]);
end
end

function J = finite_difference(fun, x, h)
% simple finite-difference derivatives
% assumes the function fun() is vectorized

if nargin < 3
    h = 2^-17;
end

[n, K]  = size(x);
H       = [zeros(n,1) h*eye(n)];
H       = permute(H, [1 3 2]);
X       = pp(x, H);
X       = reshape(X, n, K*(n+1));
Y       = fun(X);
m       = numel(Y)/(K*(n+1));
Y       = reshape(Y, m, K, n+1);
J       = pp(Y(:,:,2:end), -Y(:,:,1)) / h;
J       = permute(J, [1 3 2]);
end

% utility functions: singleton-expanded addition and multiplication
function c = pp(a,b)
c = bsxfun(@plus,a,b);
end

function c = tt(a,b)
c = bsxfun(@times,a,b);
end