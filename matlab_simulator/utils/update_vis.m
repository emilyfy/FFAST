function vis = update_vis(vis, X, U, obs, path)
% Update visualization with current state
% vis_struct = update_vis(vis_struct, X, U, obs, path)
%
% X = [x;y;psi;v_x;v_y;psi_dot]
% U = [wheel_speed;steer_angle]
% obs = [x;y] for upward facing or [x;y;psi] to set orientation (optional)
% path = [ X_k X_k+1 ... ] (optional)

% ----------------------------------------
% ----------Update Visualization----------
% ----------------------------------------

% get current position
pos_x = X(1);
pos_y = X(2);
pos_psi = wrapToPi(X(3));
steer = U(2);

% construct transform matrix
pos_tf = [cos(pos_psi) -sin(pos_psi) pos_x;
          sin(pos_psi) cos(pos_psi) pos_y;
          0 0 1];
steer_tf = [cos(steer) -sin(steer) 0;
            sin(steer) cos(steer) 0;
            0 0 1];

% current plot
pos_body = pos_tf * vis.body_polygon;
pos_CoG = pos_tf * vis.CoG;
pos_rear = pos_tf * vis.r_axle; 

clearpoints(vis.body_plot);
addpoints(vis.body_plot,pos_body(1,:),pos_body(2,:));

if isfield(vis,'show_traj_cog') && vis.show_traj_cog
    addpoints(vis.traj_cog,pos_CoG(1,:),pos_CoG(2,:));
end

if isfield(vis,'show_traj_r') && vis.show_traj_r
    addpoints(vis.traj_r,pos_rear(1,:),pos_rear(2,:));
end

if isfield(vis,'show_wheels') && vis.show_wheels
    clearpoints(vis.fr_plot);
    clearpoints(vis.fl_plot);
    clearpoints(vis.rr_plot);
    clearpoints(vis.rl_plot);
    
    pos_fr = pos_tf * vis.fr_tf * steer_tf * vis.wheel_polygon;
    pos_fl = pos_tf * vis.fl_tf * steer_tf * vis.wheel_polygon;
    pos_rr = pos_tf * vis.rr_polygon;
    pos_rl = pos_tf * vis.rl_polygon;
    
    addpoints(vis.fr_plot, pos_fr(1,:), pos_fr(2,:))
    addpoints(vis.fl_plot, pos_fl(1,:), pos_fl(2,:))
    addpoints(vis.rr_plot, pos_rr(1,:), pos_rr(2,:))
    addpoints(vis.rl_plot, pos_rl(1,:), pos_rl(2,:))
end

if nargin > 3
    if nargin == 4 && size(obs,1) == size(X,1) && size(obs,3) > 1
        path = obs;
        clear obs
        if isfield(vis,'path_plot')
            for j=1:size(vis.path_plot,2)
                delete(vis.path_plot(j));
            end
        end
        for j=1:size(path,2)-1
            vis.path_plot(j) = plot([path(1,j) path(1,j+1)],[path(2,j),path(2,j+1)],'Color',vis.path_color);
        end
    elseif nargin == 4 && size(obs,1) <= 3
        if size(obs,1) == 2
            obs(3) = pi/2;
        end
        if isfield(vis,'obs_plot')
            delete(vis.obs_plot)
        end
        obs_tf = [cos(obs(3)) -sin(obs(3)) obs(1);
                sin(obs(3)) cos(obs(3)) obs(2);
                0 0 1];
        obs_pos = obs_tf * vis.body_polygon;
        vis.obs_plot = fill(obs_pos(1,:),obs_pos(2,:),vis.obs_color,'EdgeColor','none');
    elseif nargin == 5
        if size(obs,1) == 2
            obs(3) = pi/2;
        end
        if isfield(vis,'obs_plot')
            delete(vis.obs_plot)
        end
        obs_tf = [cos(obs(3)) -sin(obs(3)) obs(1);
                sin(obs(3)) cos(obs(3)) obs(2);
                0 0 1];
        obs_pos = obs_tf * vis.body_polygon;
        vis.obs_plot = fill(obs_pos(1,:),obs_pos(2,:),vis.obs_color,'EdgeColor','none');

        if isfield(vis,'path_plot')
            for j=1:size(vis.path_plot,2)
                delete(vis.path_plot(j));
            end
        end
        for j=1:size(path,2)-1
            vis.path_plot(j) = plot([path(1,j) path(1,j+1)],[path(2,j),path(2,j+1)],'Color',vis.path_color);
        end
    end
end

if isfield(vis,'follow') && vis.follow
    axis(vis.ax,[pos_x-vis.x_clearance, pos_x+vis.x_clearance, pos_y-vis.y_clearance, pos_y+vis.y_clearance])
end

drawnow

if isfield(vis,'m')
    vis.m = [vis.m getframe(vis.ax)];
end

end