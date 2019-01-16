%% Author: Pierre-Yves Lajoie (lajoie.py@gmail.com)

%% Global initialization
clear all
close all
clc

%% Settings
dataset_folder = horzcat(pwd, '/../test_data/pairwise_consistency_maximization/simulation/');
number_of_robots = 2; % Only 2 is supported.
id_offset = 96; % letter a = 97 (ASCII)
sigma_R = 0.01;
sigma_t = 0.1;
trajectory_size = 20;
number_of_separators = 10;
percentage_of_outliers = 0;% Not supported yet
trajectory_offsets = {[0; 0; 0], [(rand*10)-5; (rand*10)-5; (rand*10)-5]};
use_rotation = true;

%% Setup
addpath(genpath('./posegraph_utils'));
example_folder = horzcat(dataset_folder, 'example_', num2str(number_of_robots), 'robots/');
mkdir(example_folder);
information_matrix = eye(6);
information_matrix(1:3, 1:3) = information_matrix(1:3, 1:3)*(1/(sigma_R^2));
information_matrix(4:6, 4:6) = information_matrix(4:6, 4:6)*(1/(sigma_t^2));

%% Generate file names
file_names = {};
for robot=1:number_of_robots
    file_names{end+1} = horzcat(example_folder,num2str(robot)-1,'.g2o');
end

%% Generate trajectories
robot_poses = {};
for robot=1:number_of_robots
    robot_offset = bitshift(uint64(robot+id_offset), 56); % GTSAM format
    [poses, measurements, edges_id] = generateTrajectory(robot_offset, trajectory_size, trajectory_offsets{robot}, information_matrix, use_rotation);
    poses_to_write = poses;
    for i=1:size(poses_to_write,2)
        poses_to_write(i).t = poses_to_write(i).t - trajectory_offsets{robot};
    end
    writeG2oDataset3D(file_names{robot}, measurements, edges_id, poses_to_write, robot_offset)
    robot_poses{end+1} = poses;
end

%% Add separators
robot1_offset = bitshift(uint64(1+id_offset), 56); % GTSAM format
robot2_offset = bitshift(uint64(2+id_offset), 56); % GTSAM format
[measurements, edges_id] = generateSeparators(robot_poses, robot1_offset, robot2_offset, number_of_separators, trajectory_size, sigma_R, sigma_t, information_matrix);
for robot=1:number_of_robots
    writeG2oDataset3D(file_names{robot}, measurements, edges_id, [], 0, 1);
end