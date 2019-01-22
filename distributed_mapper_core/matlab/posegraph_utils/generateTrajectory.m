function [poses, measurements, edges_id] = generateTrajectory(node_offset, trajectory_size, trajectory_offset, information_matrix, use_rotation)

poses(1).t = [0; 0; 0] + trajectory_offset(1:3);
poses(1).R = rotx(trajectory_offset(4))*roty(trajectory_offset(5))*rotz(trajectory_offset(6));
measurements.between(1).R = eye(3);
measurements.between(1).t = [0; 0; 0];
measurements.between(1).Info = information_matrix;
edges_id = uint64([]);
t_speed = 1;
for i=1:trajectory_size
    if use_rotation
        offset.R = rotx(360*rand)*roty(360*rand)*rotz(360*rand);
    else
        offset.R = eye(3);
    end
    offset.t = [2*t_speed*rand-t_speed; 2*t_speed*rand-t_speed; 2*t_speed*rand-t_speed];
    measurements.between(i).R = offset.R;
    measurements.between(i).t = offset.t;
    measurements.between(i).Info = information_matrix;
    [new_pose.t, new_pose.R] = poseAdd3D(poses(end), offset);
    new_pose.t = new_pose.t;
    poses(i+1) = new_pose;
    edges_id(end+1, :) = [uint64(node_offset+i), uint64(node_offset+i+1)];
end

end

