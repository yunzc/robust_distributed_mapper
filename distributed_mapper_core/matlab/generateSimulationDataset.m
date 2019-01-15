%% Author: Pierre-Yves Lajoie (lajoie.py@gmail.com)

%% Global initialization
clear all
close all
clc

%% Settings
dataset_folder = horzcat(pwd, '/../sim_data/');
number_of_robots = 2; % Only 2 is supported.
id_offset = 1000;
sigma_R = 0.01;
sigma_t = 0.1;
trajectory_size = 20;
number_of_separators = 10;
percentage_of_outliers = 0;% Not supported yet

%% Setup
addpath(genpath('./posegraph_utils'));
example_folder = horzcat(dataset_folder, 'example_', num2str(number_of_robots), 'robots/');
mkdir(example_folder);

%% Generate file names
file_names = {};
for robot=1:number_of_robots
    file_names{end+1} = horzcat(example_folder,num2str(robot)-1,'.g2o');
end

%% Generate trajectories
robot_poses = {};
for robot=1:number_of_robots
    [poses, measurements, edges_id] = generateTrajectory(robot*id_offset, trajectory_size);
    writeG2oDataset3D(file_names{robot}, measurements, edges_id, poses, robot*id_offset)
    robot_poses{end+1} = poses;
end

%% Add separators
[poses, measurements, edges_id] = generateSeparators(robot_poses, id_offset, 2*id_offset, number_of_separators, trajectory_size, sigma_R, sigma_t);
separator_file_name = horzcat(example_folder,'separators.g2o');
writeG2oDataset3D(separator_file_name, measurements, edges_id, poses, 0);