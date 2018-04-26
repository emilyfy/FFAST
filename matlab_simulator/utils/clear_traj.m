function vis = clear_vis(vis)
% Clear visualization trajectories
% vis_struct = clear_vis(vis_struct)

if vis.show_traj_cog
    clearpoints(vis.traj_cog);
end

if vis.show_traj_r
    clearpoints(vis.traj_r);
end

end