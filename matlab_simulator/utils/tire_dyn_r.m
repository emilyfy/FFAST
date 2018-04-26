function [Fx,Fy] = tire_dyn_r(v_x, wheel_vx, alpha)
% Rear tire dynamics
% [F_xr, F_yr] = tire_dyn_r(v_x, wheel_speed, alpha_r)

    global vehicle
    global debug

    % find longitudinal wheel slip K (kappa)
    if abs(wheel_vx-v_x) < 0.01 || (abs(wheel_vx) < 0.01 && abs(v_x) < 0.01)
        K = 0;
    elseif abs(v_x) < 0.01   % infinite slip, longitudinal saturation
        K = inf;
        Fx = sign(wheel_vx)*vehicle.mu*vehicle.load_r;
        Fy = 0;
        return
    else
        K = (wheel_vx-v_x)/abs(v_x);
    end
    if isfield(debug,'K')
        debug.K = K;
    end
    
    % instead of avoiding -1, now look for positive equivalent
    if K < 0
        spin_dir = -1;
        K = abs(K);
    else
        spin_dir = 1;
    end
    
    % alpha > pi/2 cannot be adapted to this formula
    % because of the use of tan(). Use the equivalent angle instead.
    % alpha > pi/2 means vehicle moving backwards
    % Fy sign has to be reversed, but the *sign(alpha) will take care of it
    if abs(alpha) > pi/2
        alpha = (pi-abs(alpha))*sign(alpha);
    end
 
    gamma = sqrt( vehicle.C_x^2 * (K/(1+K))^2 + vehicle.C_alpha^2 * (tan(alpha)/(1+K))^2 );
    
    if gamma <= 3*vehicle.mu*vehicle.load_r
        % F = gamma - 1/(3*vehicle.mu*vehicle.load_r)*(2-vehicle.mu_slide/vehicle.mu)*gamma^2 ...
        %     + 1/(9*vehicle.mu^2*vehicle.load_r^2)*(1-(2/3)*(vehicle.mu_slide/vehicle.mu))*gamma^3;
        F = gamma - 1/(3*vehicle.mu*vehicle.load_r)*gamma^2 + 1/(27*vehicle.mu^2*vehicle.load_r^2)*gamma^3;
        if isfield(debug,'rts')
            debug.rts = 0;
        end
    else
        % more accurate modeling with peak friction value
        F = vehicle.mu_slide*vehicle.load_r;
        if isfield(debug,'rts')
            if abs(v_x)>0.01
                debug.rts = 1;
            else
                debug.rts = 0;
            end
        end
    end
    
    if gamma == 0
        Fx = 0;
        Fy = 0;
    else
        Fx = vehicle.C_x/gamma * (K/(1+K)) * F * spin_dir;
        Fy = -vehicle.C_alpha/gamma * (tan(alpha)/(1+K)) * F;
    end
    
end