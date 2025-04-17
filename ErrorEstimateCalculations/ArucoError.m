clc; clear all; close all

testNo = 1;
testSelect = {"Old static",...
            "Static - Central", ...
            "Static - Edge", ...
            "Depth", ...
            "XY", ...
            "Rotations" ...
            };

% load rosbag
    % https://au.mathworks.com/help/ros/ref/ros2bagreader.readmessages.html
    % https://au.mathworks.com/help/ros/ref/readmessages.html?searchHighlight=readmessages&s_tid=srchtitle_support_results_1_readmessages
% bagReader = ros2bagreader(folderPath);
% optiTopic = select(bagReader, "Topic", "/vrpn_client_node/tronpi21/pose");
% optiMsgs = readMessages(optiTopic);

N = 10; % size of rosbag
% get time rosbag variable
t = [];
% get optitrack BASE LINK transform
true_xyz = nan(N, 3);
% get optitrack BASE LINK rotation matrix
true_rpy = nan(N, 3);
% get rover_centre output
estimate_xyz = nan(N, 3);
estimate_rpy = nan(N, 3);

%% If any NaN values for time, fix by setting to 0 or NAN

%% Calculate error
% Calculate xyz error
err_x = abs(true_xyz(:,1) - estimate_xyz(:,1));
err_y = abs(true_xyz(:,2) - estimate_xyz(:,2));
err_z = abs(true_xyz(:,3) - estimate_xyz(:,3));

EucError = sqrt(err_x.^2 + err_y.^2 + err_z.^2);


% Calculate rpy error
err_r = abs(true_rpy(:,1) - estimate_rpy(:,1));
err_p = abs(true_rpy(:,2) - estimate_rpy(:,2));
err_y = abs(true_rpy(:,3) - estimate_rpy(:,3));

%% FIGURES
% Euclidean Distance Error of Localisation System - [test name]
figure(1), clf
title(sprintf("Euclidean Distance Error of Localisation System - %s", testSelect{testNo}))
grid on, hold on
plot()