function joyCallback(sub, msg)

global joy cmd_vel steer

joy.stop = msg.Buttons(joy.matlab_stop_button);
joy.reset = msg.Buttons(joy.reset_button);

if msg.Buttons(joy.activate_button)
    joy.active = ~joy.active;
end

if joy.active
    if double(msg.Axes(joy.throttle_axis)) > 0
        cmd_vel = double(msg.Axes(joy.throttle_axis))^3*joy.max_vel;
    elseif double(msg.Axes(joy.throttle_axis)) <0
        cmd_vel = double(msg.Axes(joy.throttle_axis))^3*-1*joy.min_vel;
    else
        cmd_vel = 0;
    end
    
    if abs(cmd_vel) < joy.min_speed
        cmd_vel = 0;
    end
    
    steer = joy.max_steer*double(msg.Axes(joy.steer_axis));

    if msg.Buttons(joy.brake_button)
        cmd_vel = 0;
    end
else
    cmd_vel = 0;
    steer = 0;
end

if msg.Buttons(joy.estop_button)
    cmd_vel = 0;
    steer = 0;
end

end
