function plt = init_plot(plt)
% Initialize plot
% plt_struct = init_plot(plt_struct)
%
% plt struct members:
% ------------------
% titles  - array of val names
% color   - plot color (default black)
% style   - plot line style (default solid line)
% vals    - array of plot values
% dt      - time between each call to update_plot (default measures actual time)
% keepfig - set to number of figure to plot on that figure instead of opening new one

if isfield(plt,'keepfig')
    plt.fig = figure(plt.keepfig);
else
    plt.fig = figure();
end

set(plt.fig,'Name','Plot of values');
n = numel(plt.titles);
plt.plots = [];
plt.axes = [];

plt.curr_time = 0;
plt.start_time = tic;

if ~isfield(plt,'color')
    plt.color = 'k';
end
if ~isfield(plt,'style')
    plt.style = '-';
end

for i=1:n
    plt.axes = [plt.axes subplot(n,1,i)];
    plt.plots = [plt.plots animatedline('Color',plt.color,'LineStyle',plt.style)];
    title(plt.titles(i));
end
end
