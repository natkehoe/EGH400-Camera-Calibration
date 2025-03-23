clc, clear all, close all

%% ROS2 BAG READER
% Try 1 at getting messages from rosbag2.

% Get bag location
if (~exist('bagReader', 'var'))
    path = "C:\Users\nhkje\EGH400\Rosbag conversion\ros2_bag_dir\ros2_bag_dir.db3";
    bagReader = ros2bagreader(path);
    % disp("True")
end

% read messages
msgs = readMessages(bagReader);