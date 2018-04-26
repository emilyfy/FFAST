function poseCallback(sub, msg)
global ros_X
ros_euler = quat2eul([msg.Pose.Pose.Orientation.W msg.Pose.Pose.Orientation.X ...
                      msg.Pose.Pose.Orientation.Y msg.Pose.Pose.Orientation.Z]);
ros_X = [msg.Pose.Pose.Position.X;
         msg.Pose.Pose.Position.Y;
         ros_euler(1);
         0; 0; 0];
%ros_X = double([msg.X; msg.Y; msg.Theta]);
end