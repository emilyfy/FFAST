function plt = update_plot(plt)
% Update plot with current value
% plt_struct = update_plot(plt_struct)
%
% plt struct members:
% ------------------
% vals - array of plot values
% dt   - time between each call to update_plot (default measures actual time)

n = numel(plt.plots);

if isfield(plt,'dt')
    plt.curr_time = plt.curr_time + plt.dt;
else
    plt.curr_time = toc(plt.start_time);
end

for i=1:n
    addpoints(plt.plots(i),plt.curr_time,plt.vals(i));
    axis(plt.axes(i),[plt.curr_time-20, plt.curr_time+1, -inf, inf])
end
drawnow

if isfield(plt,'m')
    plt.m = [plt.m getframe(plt.fig)];
end

end
