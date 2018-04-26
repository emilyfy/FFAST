function vis = init_vis(vis)
% Initialize visualization
% vis_struct = init_vis(vis_struct)
%
% vis struct members:
% ------------------
% color         - vehicle color (default black)
% lane_lines    - array of lane lines [x1 x2 y1 y2] to be plotted
% lane_color    - color of lane lines (default black)
% obs_color     - obstacle color (default black)
% path_color    - path color (default green)
% show_traj_cog - show trajectory of CoG
% cog_color     - color of CoG trajectory (default black)
% cog_style     - line style of CoG trajectory (default solid line)
% show_traj_r   - show trajectory of rear axle
% rax_color     - color of rear axle trajectory (default black)
% rax_style     - line style of rear axle trajectory (default solid line)
% show_wheels   - show vehicle wheels in visualization
% show_grid     - show grid
% keepfig       - set to number of figure to plot on that figure instead of opening new one
% axlims        - manually set axes limits
% follow        - set to 1 to automatically set axes to follow vehicle position
% x_clearance   - clearance between vehicle CoG and x axis boundaries (necessary if follow is true)
% y_clearance   - clearance between vehicle CoG and y axis boundaries (necessary if follow is true)

% ----------------------------------------
% --------Initialize Visualization--------
% ----------------------------------------

if isfield(vis,'keepfig')
    vis.fig = figure(vis.keepfig);
else
    vis.fig = figure();
end

set(vis.fig,'Name','Vehicle simulator');
vis.ax = gca();
hold(vis.ax,'all')
box(vis.ax,'on')
axis auto equal

if isfield(vis,'show_grid') && vis.show_grid
    grid(vis.ax,'on')
end

if ~isfield(vis,'color')
    vis.color = 'k';
end
if ~isfield(vis,'obs_color')
    vis.obs_color = 'k';
end
if ~isfield(vis,'path_color')
    vis.path_color = 'g';
end

if isfield(vis,'lane_lines')
    for i=1:size(vis.lane_lines,1)
        if ~isfield(vis,'lane_color')
            vis.lane_color = 'k';
        end
        vis.lanes_plot(i) = plot([vis.lane_lines(i,1) vis.lane_lines(i,2)],[vis.lane_lines(i,3) vis.lane_lines(i,4)],vis.lane_color);
    end
end

global vehicle

% vehicle body
vis.body_polygon = [-vehicle.L_r  -vehicle.L_r  vehicle.L_f  vehicle.L_f  -vehicle.L_r;
                     -vehicle.tw/2 vehicle.tw/2  vehicle.tw/2 -vehicle.tw/2 -vehicle.tw/2;
                     1 1 1 1 1];
vis.body_plot = animatedline(vis.body_polygon(1,:),vis.body_polygon(2,:),'Color',vis.color);

% trajectories
vis.CoG = [0;0;1];
vis.r_axle = [-vehicle.L_r;0;1];

if isfield(vis,'show_traj_cog') && vis.show_traj_cog
    if ~isfield(vis,'cog_color')
        vis.cog_color = 'k';
    end
    if ~isfield(vis,'cog_style')
        vis.cog_style = '-';
    end
    vis.traj_cog = animatedline(vis.CoG(1,:),vis.CoG(2,:),'Color',vis.cog_color,'LineStyle',vis.cog_style);
end

if isfield(vis,'show_traj_r') && vis.show_traj_r
    if ~isfield(vis,'rax_color')
        vis.rax_color = 'k';
    end
    if ~isfield(vis,'rax_style')
        vis.rax_style = '-';
    end
    vis.traj_r = animatedline(vis.r_axle(1,:),vis.r_axle(2,:),'Color',vis.rax_color,'LineStyle',vis.rax_style);
end

% wheels
if isfield(vis,'show_wheels') && vis.show_wheels
    vis.wheel_polygon = [-vehicle.wheel_dia -vehicle.wheel_dia vehicle.wheel_dia vehicle.wheel_dia -vehicle.wheel_dia;
                          -vehicle.wheel_w vehicle.wheel_w vehicle.wheel_w -vehicle.wheel_w -vehicle.wheel_w;
                          1 1 1 1 1];
    
    % translation matrix to wheel positions
    vis.fr_tf = [1 0 vehicle.L_f; 0 1 -vehicle.tw/2; 0 0 1];
    vis.fl_tf = [1 0 vehicle.L_f; 0 1 vehicle.tw/2 ; 0 0 1];
    vis.rr_tf = [1 0 -vehicle.L_r; 0 1 -vehicle.tw/2; 0 0 1];
    vis.rl_tf = [1 0 -vehicle.L_r; 0 1 vehicle.tw/2 ; 0 0 1];
    
    fr_polygon = vis.fr_tf * vis.wheel_polygon;
    fl_polygon = vis.fl_tf * vis.wheel_polygon;
    vis.rr_polygon = vis.rr_tf * vis.wheel_polygon;
    vis.rl_polygon = vis.rl_tf * vis.wheel_polygon;
    
    vis.fr_plot = animatedline(fr_polygon(1,:),fr_polygon(2,:),'Color',vis.color);
    vis.fl_plot = animatedline(fl_polygon(1,:),fl_polygon(2,:),'Color',vis.color);
    vis.rr_plot = animatedline(vis.rr_polygon(1,:),vis.rr_polygon(2,:),'Color',vis.color);
    vis.rl_plot = animatedline(vis.rl_polygon(1,:),vis.rl_polygon(2,:),'Color',vis.color);
end

if isfield(vis,'follow') && vis.follow
    axis(vis.ax,[-vis.x_clearance, vis.x_clearance, -vis.y_clearance, vis.y_clearance])
else
    if isfield(vis,'axlims')
        axis(vis.ax,'manual')
        axis(vis.ax,vis.axlims)
    end
end

end