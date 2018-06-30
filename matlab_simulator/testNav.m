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

p.cu = 1e-3*[1 0];
p.cdu = 1e-3*[1 6];

cf_ = [3 3 10 1 1 .1];
p.cf = [0 0 0 0 0 0];
p.pf = [ .1 .1 .1 .01 .01 .01];

p.cx  = 1;
p.px  = .01;
p.ca = 0.1;

p.seg_obs_thres = 0.5;
p.cir_obs_thres = 0.5;
p.cco = 0;
p.cso = 0;
p.cir_obs = [0 0];
p.seg_obs = [10 10 20 20];

Op.max_iter = 50;
T = 20;

% initial state
x0 = [0;0;0;.1;0;0;0;0];
x = x0(1:6);

% plot
vis.show_wheels = 1;
vis.show_grid = 1;
vis.show_traj_cog = 1;
vis.axlims = [-5 5 -5 5];
vis.m = [];
vis = init_vis(vis);
set(vis.ax,'xtick',-5:5,'ytick',-5:5)

% variables to be edited in callback functions
obs.cir = []; % x y rad
obs.seg = []; % x1 y1 x2 y2
goal = []; % x y th

vis.cir_obs_plot = [];
vis.seg_obs_plot = [];
goal_reached = 1;

setappdata(vis.fig,'ocbtn',0)
setappdata(vis.fig,'osbtn',0)
setappdata(vis.fig,'gbtn',0)
setappdata(vis.fig,'rbtn',0)
setappdata(vis.fig,'qbtn',0)
setappdata(vis.fig,'vbtn',0)

vis.ocbtn = uicontrol('Style', 'pushbutton', 'String', 'add circ obs',...
        'Position', [10 10 100 20], 'Callback', @ocCb);
vis.osbtn = uicontrol('Style', 'pushbutton', 'String', 'add seg obs',...
        'Position', [120 10 100 20], 'Callback', @osCb);
vis.gbtn = uicontrol('Style', 'pushbutton', 'String', 'set goal',...
       'Position', [230 10 70 20], 'Callback', @gCb);
vis.rbtn = uicontrol('Style', 'pushbutton', 'String', 'reset',...
        'Position', [310 10 50 20], 'Callback', @rCb);
vis.gbtn = uicontrol('Style', 'pushbutton', 'String', 'visualize',...
       'Position', [370 10 70 20], 'Callback', @vCb);
vis.gbtn = uicontrol('Style', 'pushbutton', 'String', 'quit',...
       'Position', [450 10 50 20], 'Callback', @qCb);
drawnow

while 1
    % check for button push
    pause(0.1)
    if getappdata(vis.fig,'qbtn')
        close(vis.fig)
        break
    end
    if getappdata(vis.fig,'ocbtn')
        [vis, obs] = setCirObs(vis, obs);
    end
    if getappdata(vis.fig,'osbtn')
        [vis, obs] = setSegObs(vis, obs);
    end
    if getappdata(vis.fig,'rbtn')
        [vis,obs] = resetFig(vis, obs);
        x0 = [0;0;0;.1;0;0;0;0];
        x = x0(1:6);
    end
    if getappdata(vis.fig,'vbtn')
        vis = vis_seq(vis, seq);
        setappdata(vis.fig,'vbtn',0)
    end
    if getappdata(vis.fig,'gbtn')
        [vis,goal] = setGoal(vis,goal);

        p.goal = [goal 0 0 0];
        p = checkObs(p,x,obs);
        goal_reached = 0;
        step = 0;
        seq.X = [];
        seq.U = [];

        % nominal sequence
        del_th = wrapToPi(goal(3)-x(3));
        u0 = [1+.2*randn(1,T); .01*del_th+.05*randn(1,T) ];
    end
    
    % plan
    while ~goal_reached && step<1000
        % check for quit botton push
        pause(0.01)
        if getappdata(vis.fig,'qbtn')
            break
        end
        
        a = tic; [success, xx, uu, cost] = iLQGNav(x0, u0, p, Op); b = toc(a);
        
        if b <= T*p.dt
            for i=1:ceil(b/p.dt)
                x_new = state_transition(x,uu(:,i),p.dt,p);
                if ~checkColl(x_new,obs)
                    x = x_new;
                end
                [seq.X,seq.U,step,goal_reached,p] = updateVars(seq.X,seq.U,step,x,uu(:,i),goal,p,cf_);
                p = checkObs(p,x,obs);
            end
            ulast = uu(:,i);
            u0 = [uu(:,i+1:end) [1+.2*randn(1,i) ; .1*randn(1,i)] ];
        else
            for i=1:T
                x_new = state_transition(x,uu(:,i),p.dt,p);
                if ~checkColl(x_new,obs)
                    x = x_new;
                end
                [seq.X,seq.U,step,goal_reached,p] = updateVars(seq.X,seq.U,step,x,uu(:,i),goal,p,cf_);
                p = checkObs(p,x,obs);
            end
            for i=T+1:ceil(b/p.dt)
                x_new = state_transition(x,[0;0],p.dt,p);
                if ~checkColl(x_new,obs)
                    x = x_new;
                end
                [seq.X,seq.U,step,goal_reached,p] = updateVars(seq.X,seq.U,step,x,uu(:,i),goal,p,cf_);
                p = checkObs(p,x,obs);
            end
            ulast = [0;0];
            u0 = [1+.2*randn(1,T) ; .1*randn(1,T)];
        end

        x0 = [x(1:6) ; ulast(1) ; ulast(2) ];

    end
    
end

function [X,U,step,goal_reached,p] = updateVars(X,U,step,x,u,goal,p,cf_)
    X = [X x];
    U = [U u];
    step = step + 1;
    
    goal_reached = 0;
    if sqrt( (x(1)-goal(1))^2.0 + (x(2)-goal(2))^2.0 ) < 0.2 && abs(x(3)-goal(3)) < 0.3
        goal_reached = 1;
    end
    
    if sqrt( (x(1)-goal(1))^2.0 + (x(2)-goal(2))^2.0 ) < 1 && abs(x(3)-goal(3)) < 1
        p.cf = [0 0 0 0 0 0];
    else
        p.cf = cf_;
    end
end

function p = checkObs(p,x,obs)
    % find closest cir obs
    cir_dist = 100;
    cir_idx = 1;
    for i=1:size(obs.cir,1)
        dist = sqrt( (x(1)-obs.cir(i,1))^2 + (x(2)-obs.cir(i,2))^2 );
        if dist < cir_dist
            cir_dist = dist;
            cir_idx = i;
        end
    end
    
    % find closest seg obs
    seg_dist = 100;
    seg_idx = 1;
    for i=1:size(obs.seg,1)
        dist = segDist(x(1:2),obs.seg(i,:));
        if dist < seg_dist
            seg_dist = dist;
            seg_idx = i;
        end
    end
    
    % update p
    if length(obs.cir) == 0
        p.cco = 0;
    else
        p.cir_obs_thres = 0.6 + obs.cir(cir_idx,3); % rad+0.6 m
        p.cco = 5 + obs.cir(cir_idx,3);
        p.cir_obs = obs.cir(cir_idx,1:2);
    end
    if length(obs.seg) == 0
        p.cso = 0;
    else
        p.cso = 5/velToSeg(x,obs.seg(seg_idx,:));
        p.seg_obs = obs.seg(seg_idx,:);
    end
    
    if cir_dist < p.cir_obs_thres || seg_dist < p.seg_obs_thres
        p.cx = 0;
        p.ca = 0;
        p.cf = [ 0 0 0 0 0 0];
    else
        p.cx  = 1;
        p.ca = 0.1;
    end
    
    function dist = segDist(x,seg)
        vecdist = @(v, w) sqrt( (v(1)-w(1))^2 + (v(2)-w(2))^2 );
        l = vecdist(seg(1:2),seg(3:4));
        if l == 0
            dist = vecdist(x,seg(1:2));
        else
            t = ((x(1) - seg(1)) * (seg(3) - seg(1)) + (x(2) - seg(2)) * (seg(4) - seg(2))) / l^2;
            t = max(0,min(1,t));
            dist = vecdist(x,[seg(1)+t*(seg(3)-seg(1)) seg(2)+t*(seg(4)-seg(2))]);
        end
    end
    
    function dist = velToSeg(x,seg)
        % stackoverflow.com/questions/563198/
        p_ = [x(1:2)];
        r = [x(4:5)];
        q = [seg(1:2)];
        s = [seg(3:4)-seg(1:2)];
        
        cross2d = @(v,w) v(1)*w(2)-v(2)*w(1);
        pq = q-p_;
        rs = cross2d(r,s);
        t = cross2d(pq,s)/rs;
        u = cross2d(pq,r)/rs;
        
        if rs == 0 || u<-0.05 || u>1.05
            dist = inf;
        else
            dist = abs(t)*norm(r);
        end
    end
end

function coll = checkColl(X, obs)
    global vehicle
    coll = 0;
    car_polygon = [cos(X(3)) -sin(X(3)) X(1); ...
                   sin(X(3)) cos(X(3)) X(2); ...
                   0 0 1] * ...
                  [-vehicle.L_r  -vehicle.L_r  vehicle.L_f  vehicle.L_f  -vehicle.L_r; ...
                   -vehicle.tw/2 vehicle.tw/2  vehicle.tw/2 -vehicle.tw/2 -vehicle.tw/2; ...
                   1 1 1 1 1];
    
    for i=1:size(obs.cir,1)
        overlap = 1;
        ax = obs.cir(i,1:2) - X(1:2)';
        x_min = 50; x_max = -50;
        for j=1:4
            vert = car_polygon(1:2,j);
            proj = dot(vert,ax);
            if proj < x_min
                x_min = proj;
            end
            if proj > x_max
                x_max = proj;
            end
        end
        proj = dot(obs.cir(i,1:2),ax);
        obs_min = proj - obs.cir(i,3);
        obs_max = proj + obs.cir(i,3);
        
        if x_max <= obs_min || x_min >= obs_max
            overlap = 0;
        else
            coll = 1;
            return
        end
    end
    
    for i=1:size(obs.seg,1)
        overlap = 1;
        axes = [obs.seg(i,3)-obs.seg(i,1) obs.seg(i,4)-obs.seg(i,2) ;
                obs.seg(i,4)-obs.seg(i,2) obs.seg(i,1)-obs.seg(i,3) ;
                cos(X(3)) sin(X(3)) ;
                -sin(X(3)) cos(X(3)) ];
        obs_verts = [obs.seg(i,1:2) ; obs.seg(i,3:4) ];
        car_verts = car_polygon(1:2,1:4)';
        for j=1:size(axes,1)
            ax = axes(j,:);
            x_min = 50; x_max = -50;
            obs_min = 50; obs_max = -50;
            for k=1:size(obs_verts,1)
                vert = obs_verts(k,:);
                proj = dot(vert,ax);
                if proj < obs_min
                    obs_min = proj;
                end
                if proj > obs_max
                    obs_max = proj;
                end
            end
            for k=1:size(car_verts,1)
                vert = car_verts(k,:);
                proj = dot(vert,ax);
                if proj < x_min
                    x_min = proj;
                end
                if proj > x_max
                    x_max = proj;
                end
            end

            if x_max <= obs_min || x_min >= obs_max
                overlap = 0;
            end
        end
        if overlap == 1
            coll = 1;
            return
        end
    end
end

function [vis, obs] = setCirObs(vis,obs)
    set(vis.fig,'Pointer','hand')
    click = [];
    set(vis.fig,'WindowButtonDownFcn',@clickDown)
    set(vis.fig, 'WindowButtonUpFcn',@clickUp)
    
    while length(click)<4 && getappdata(vis.fig,'ocbtn')
        pause(0.1)
    end
    
    x = click(1);
    y = click(2);
    rad = sqrt( (click(1)-click(3))^2 + (click(2)-click(4))^2 );
    obs.cir = [obs.cir; [x y rad] ];
    
    set(vis.fig,'Pointer','arrow')
    vis.cir_obs_plot = [vis.cir_obs_plot rectangle('Position',[x,y,2*rad,2*rad],'Curvature',[1 1],'FaceColor','k')];
    setappdata(vis.fig,'ocbtn',0)
    
    function clickDown (gcbo,eventdata)
        c = get(vis.ax,'CurrentPoint');
        click(1,1:2) = c(1,1:2);
    end

    function clickUp (gcbo,eventdata)
        c = get(vis.ax,'CurrentPoint');
        click(1,3:4) = c(1,1:2);
    end
end

function [vis, obs] = setSegObs(vis,obs)
    set(vis.fig,'Pointer','hand')
    click = [];
    set(vis.fig,'WindowButtonDownFcn',@clickDown)
    set(vis.fig, 'WindowButtonUpFcn',@clickUp)
    
    while length(click)<4 && getappdata(vis.fig,'osbtn')
        pause(0.1)
    end
    
    obs.seg = [obs.seg; click];
    
    set(vis.fig,'Pointer','arrow')
    vis.seg_obs_plot = [vis.seg_obs_plot plot([click(1) click(3)],[click(2) click(4)],'k-','LineWidth',5)];
    setappdata(vis.fig,'osbtn',0)
    
    function clickDown (gcbo,eventdata)
        c = get(vis.ax,'CurrentPoint');
        click(1,1:2) = c(1,1:2);
    end

    function clickUp (gcbo,eventdata)
        c = get(vis.ax,'CurrentPoint');
        click(1,3:4) = c(1,1:2);
    end
end

function [vis, obs] = resetFig(vis,obs)
    obs.cir = [];
    obs.seg = [];
    for i = length(vis.cir_obs_plot):-1:1
        delete(vis.cir_obs_plot(i))
        vis.cir_obs_plot = vis.cir_obs_plot(1:i-1);
    end
    for i = length(vis.seg_obs_plot):-1:1
        delete(vis.seg_obs_plot(i))
        vis.seg_obs_plot = vis.seg_obs_plot(1:i-1);
    end
    if isfield(vis,'goal_plot')
        delete(vis.goal_plot)
    end
    vis = update_vis(vis, [0;0;0;0;0;0], [0;0]);
    setappdata(vis.fig,'rbtn',0)
end

function [vis, goal] = setGoal(vis,goal)
    set(vis.fig,'Pointer','hand')
    click = [];
    set(vis.fig,'WindowButtonDownFcn',@clickDown)
    set(vis.fig, 'WindowButtonUpFcn',@clickUp)
    
    while length(click)<4 && getappdata(vis.fig,'gbtn')
        pause(0.1)
    end
    
    x = click(1);
    y = click(2);
    th = atan2(click(4)-click(2),click(3)-click(1));
    goal = [x y th];
    
    set(vis.fig,'Pointer','arrow')
    if isfield(vis,'goal_plot')
        delete(vis.goal_plot)
    end
    vis.goal_plot = quiver(x,y,.6*cos(th),.6*sin(th),'r','MaxHeadSize',0.8);
    setappdata(vis.fig,'gbtn',0)
    
    function clickDown (gcbo,eventdata)
        c = get(vis.ax,'CurrentPoint');
        click(1,1:2) = c(1,1:2);
    end

    function clickUp (gcbo,eventdata)
        c = get(vis.ax,'CurrentPoint');
        click(1,3:4) = c(1,1:2);
    end
end

function ocCb(source,event)
    fig = get(source,'Parent');
    ocbtn = getappdata(fig,'ocbtn');
    setappdata(fig,'ocbtn',~ocbtn)
end
function osCb(source,event)
    fig = get(source,'Parent');
    osbtn = getappdata(fig,'osbtn');
    setappdata(fig,'osbtn',~osbtn)
end
function gCb(source,event)
    fig = get(source,'Parent');
    gbtn = getappdata(fig,'gbtn');
    setappdata(fig,'gbtn',~gbtn)
end
function rCb(source,event)
    fig = get(source,'Parent');
    rbtn = getappdata(fig,'rbtn');
    setappdata(fig,'rbtn',~rbtn)
end
function vCb(source,event)
    fig = get(source,'Parent');
    vbtn = getappdata(fig,'vbtn');
    setappdata(fig,'vbtn',~vbtn)
end
function qCb(source,event)
    fig = get(source,'Parent');
    setappdata(fig,'qbtn',1)
end