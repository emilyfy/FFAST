function Fy = tire_dyn_f(alpha)
% Front tire dynamics
% F_y = tire_dyn_f(alpha_f)

    global vehicle
    
    % alpha > pi/2 cannot be adapted to this formula
    % because of the use of tan(). Use the equivalent angle instead.
    % alpha > pi/2 means vehicle moving backwards
    % Fy sign has to be reversed, but the *sign(alpha) will take care of it
    if abs(alpha) > pi/2
        alpha = (pi-abs(alpha))*sign(alpha);
    end
    
    alpha_sl = atan(3*vehicle.mu*vehicle.load_f/vehicle.C_alpha);
    if abs(alpha) <= alpha_sl
        Fy = -vehicle.C_alpha*tan(alpha) + vehicle.C_alpha^2/(3*vehicle.mu*vehicle.load_f)*abs(tan(alpha))*tan(alpha) ...
             - vehicle.C_alpha^3/(27*vehicle.mu^2*vehicle.load_f^2)*tan(alpha)^3;
    else
        Fy = -vehicle.mu*vehicle.load_f*sign(alpha);
    end
end