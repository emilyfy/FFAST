dist2goal = p.goal(1)-x(1);

if dist2goal < 0.1
    p.cf = [0 cf_(2:6)];
end

if osp == 5
    if dist2goal < 0.5
        x(7) = -1;
        p.obs_vel(2) = 0;
    elseif set_obs==0 || x(8) > x(2)+0.5 || x(7) < x(1)-0.5
        p.obs_vel(2) = 3*rand+1;
        x(7) = x(1)+1.3*rand+0.2;
        x(8) = x(2)-0.5;
        set_obs = 1;
    end
else
    if x(1) > 2 && set_obs == 0
        x(7) = x(1)+1;
        x(8) = -(osp/x(4));
        p.obs_vel(2) = osp;
        set_obs = 1;
    elseif set_obs == 1 && x(8) >= osp/vsp
        p.obs_vel(2) = 0;
    end
end

if exist('seq')
    if isfield(seq,'X')
        seq.X = [seq.X x(1:6)];
    end
    if isfield(seq,'U')
        if i > T
            seq.U = [seq.U [0;0]];
        else
            seq.U = [seq.U uu(:,i)];
        end
    end
    if isfield(seq,'obs')
        seq.obs = [seq.obs x(7:8)];
    end
    if isfield(seq,'path')
        seq.path = cat(3,seq.path,xx);
    end
    if isfield(seq,'rts')
        seq.rts = [seq.rts debug.rts];
    end
end

if exist('maxdev')
    maxdev = max(maxdev,x(2));
end
if exist('mindist')
    mindist = min(mindist,sqrt((x(1)-x(7))^2+(x(2)-x(8))^2));
end
if exist('osucc') && osucc==1 && abs(x(1)-x(7))<coll_thr && abs(x(2)-x(8))<coll_thr
    osucc = 0;
end

step = step+1;

%plt = update_plot(plt,x0(1:6),uu(:,i),x0(9:10));