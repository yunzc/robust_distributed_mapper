function [poses, measurements, edges_id] = generateTrajectory(nodeOffset, trajectory_size)

poses(1).R = eye(3);
poses(1).t = [0 0 0];
measurements.between(1).R = eye(3);
measurements.between(1).t = [0; 0; 0];
measurements.between(1).Info = eye(6);
edges_id = [];
for i=1:trajectory_size
    offset.R = rotx(rand*360)*roty(rand*360)*rotz(rand*360);
    offset.t = [rand; rand; rand];
    measurements.between(i).R = offset.R;
    measurements.between(i).t = offset.t;
    measurements.between(i).Info = eye(6);
    [new_pose.t, new_pose.R] = poseAdd3D(poses(end), offset);
    poses(i+1) = new_pose;
    edges_id(end+1, :) = [nodeOffset+i, nodeOffset+i+1];
end

end

