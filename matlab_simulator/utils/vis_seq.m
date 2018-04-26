function vis = vis_seq(vis,seq)
% Visualize whole sequence of vehicle trajectory all at once
% vis_struct = vis_seq(vis_struct,seq_struct)
% call after initialization with init_vis
%
% seq struct members: dt,X,U,obs,path,rts
% ------------------
% dt = time delay between frames (default 0.02s)
% X(:,k) = [x;y;psi;v_x;v_y;psi_dot]
% U(:,k) = [wheel_speed;steering_angle]
% obs(:,k) = [x;y] for upward facing car or [x;y;psi] to set orientation (optional)
% path(:,:,k) = [ X_k X_k+1 ... ] (optional)
% rts(:,k) = 1 if rear tire saturated, 0 if not (optional, will result in printed statements)
% at each timestep k
%
% vis struct members:
% ------------------
% m - set to empty array, the frames from the sequence will be returned as movie

N = size(seq.X,2);

seq.U(:,N)=[0;0];

if isfield(seq,'obs')
    if size(seq.obs,1) == 2
        seq.obs(3,:) = pi/2*ones(1,N);
    end
    obs_tf = [cos(seq.obs(3,1)) -sin(seq.obs(3,1)) seq.obs(1,1);
              sin(seq.obs(3,1)) cos(seq.obs(3,1)) seq.obs(2,1);
              0 0 1];
    obs_pos = obs_tf * vis.body_polygon;
    vis.obs_plot = fill(obs_pos(1,:),obs_pos(2,:),vis.obs_color,'EdgeColor',vis.obs_color);
end

if isfield(seq,'path')
    seq.path(7:end,:,:) = [];
    seq.path(:,:,N) = seq.X(:,end)*ones(1,size(seq.path,2));
    for i=1:size(seq.path,2)-1
        vis.path_plot(i) = plot([seq.path(1,i,1) seq.path(1,i+1,1)],[seq.path(2,i,1),seq.path(2,i+1,1)],'Color',vis.path_color);
    end
end  

if ~isfield(seq,'dt')
    seq.dt = 0.02;
end

load('vehicle.mat')

tic
for i=1:N
    % ----------------------------------------
    % ----------Update Visualization----------
    % ----------------------------------------
    pos_x = seq.X(1,i);
    pos_y = seq.X(2,i);
    pos_psi = wrapToPi(seq.X(3,i));
    steer = seq.U(2,i);
    
    pos_tf = [cos(pos_psi) -sin(pos_psi) pos_x;
              sin(pos_psi) cos(pos_psi) pos_y;
              0 0 1];
    steer_tf = [cos(steer) -sin(steer) 0;
                sin(steer) cos(steer) 0;
                0 0 1];
            
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

    if isfield(seq,'obs')
        obs_tf = [cos(seq.obs(3,i)) -sin(seq.obs(3,i)) seq.obs(1,i);
                sin(seq.obs(3,i)) cos(seq.obs(3,i)) seq.obs(2,i);
                0 0 1];
        obs_pos = obs_tf * vis.body_polygon;
        delete(vis.obs_plot)
        vis.obs_plot = fill(obs_pos(1,:),obs_pos(2,:),vis.obs_color,'EdgeColor','none');
    end

    if isfield(seq,'path')
        for j=1:size(vis.path_plot,2)
            delete(vis.path_plot(j));
        end
        for j=1:size(seq.path,2)-1
            vis.path_plot(j) = plot([seq.path(1,j,i) seq.path(1,j+1,i)],[seq.path(2,j,i),seq.path(2,j+1,i)],'Color',vis.path_color);
        end
    end

    if isfield(seq,'rts') && seq.rts(:,i) == 1
        fprintf('rts\n')
    end

    if isfield(vis,'follow') && vis.follow
        axis(vis.ax,[pos_x-vis.x_clearance, pos_x+vis.x_clearance, pos_y-vis.y_clearance, pos_y+vis.y_clearance])
        axis(vis.ax,'equal')
    end

    drawnow

    if isfield(vis,'m')
        vis.m = [vis.m getframe(vis.ax)];
    end

    %pause(seq.dt);
    dt = toc;tic
    pause(seq.dt-dt)
end

if isfield(seq,'obs')
    delete(vis.obs_plot)
end

end