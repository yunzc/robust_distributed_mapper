function [poses, measurements, edges_id] = generateTrajectory(node_offset, trajectory_size, trajectory_offset, information_matrix, use_rotation)

poses(1).R = eye(3);
poses(1).t = [0; 0; 0] + trajectory_offset;
measurements.between(1).R = eye(3);
measurements.between(1).t = [0; 0; 0];
measurements.between(1).Info = information_matrix;
edges_id = uint64([]);
t_speed = 1;
R_speed = 10;
for i=1:trajectory_size
    if use_rotation
        offset.R = rotx(rand*2*R_speed-R_speed)*roty(rand*2*R_speed-R_speed)*rotz(rand*2*R_speed-R_speed);
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

