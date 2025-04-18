clc; clear all; close all

testNo = 5;
testSelect = {"Old depth",...
            "Static - Central", ...
            "Static - Edge", ...
            "Depth", ...
            "XY", ...
            "Rotations" ...
            };

% add file directories
testBagFile = {...
    "", ...
    "", ...
    "", ...
    "", ...
    "C:\Users\nhkje\EGH400\ROSBAG Recordings\20250418_arucobag-xy2\20250418_arucobag-xy2_0.mcap", ...
    "" ...
};

% load rosbag
    % https://au.mathworks.com/help/ros/ref/ros2bagreader.readmessages.html
    % https://au.mathworks.com/help/ros/ref/readmessages.html?searchHighlight=readmessages&s_tid=srchtitle_support_results_1_readmessages


bagReader = ros2bagreader(testBagFile{testNo});


timeStart = bagReader.StartTime; % bag start time
optiTopic = select(bagReader, "Topic", "/vrpn_client_node/tronpi21/pose");
optiMsgs = readMessages(optiTopic);
optiMsgs_time = optiTopic.MessageList.Time - timeStart;

arucoTopic = select(bagReader, "Topic", "/rover_centre");
arucoMsgs = readMessages(arucoTopic);
arucoMsgs_time = arucoTopic.MessageList.Time - timeStart;

% get messages that have the same timestamp


N = 10; % size of rosbag
% get time rosbag variable
t = [];
% get optitrack BASE LINK transform
true_xyz = [cellfun(@(msg) msg.pose.position.x, optiMsgs), ...
            cellfun(@(msg) msg.pose.position.y, optiMsgs), ...
            cellfun(@(msg) msg.pose.position.z, optiMsgs)];
% get optitrack BASE LINK rotation matrix
true_rpy = [cellfun(@(msg) msg.pose.orientation.x, optiMsgs), ...
            cellfun(@(msg) msg.pose.orientation.y, optiMsgs), ...
            cellfun(@(msg) msg.pose.orientation.z, optiMsgs), ...
            cellfun(@(msg) msg.pose.orientation.w, optiMsgs)];
% get rover_centre output
estimate_xyz = [cellfun(@(msg) msg.pose.position.x, arucoMsgs), ...
                cellfun(@(msg) msg.pose.position.y, arucoMsgs), ...
                cellfun(@(msg) msg.pose.position.z, arucoMsgs)];
estimate_rpy = [cellfun(@(msg) msg.pose.orientation.x, arucoMsgs), ...
                cellfun(@(msg) msg.pose.orientation.y, arucoMsgs), ...
                cellfun(@(msg) msg.pose.orientation.z, arucoMsgs), ...
                cellfun(@(msg) msg.pose.orientation.w, arucoMsgs)];

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
title(sprintf("Euclidean Distance Error of Localisation System (%s)", testSelect{testNo}))
grid on, hold on
plot(t, EucError)
xlabel("Time (s)"), ylabel("Euclidean Error (m)")

    % if roll, show roll-pitch-yaw change over time

% Individual axes errors 
figure(2), clf
title(sprintf("Error in Individual Axes (%s)", testSelect{testNo}))
grid on, hold on
plot(t, err_x, 'r')
plot(t, err_y, 'g')
plot(t, err_z, 'b')
legend("X error", "Y error", "Z error")
xlabel("Time (s)"), ylabel("Euclidean Error (m)")

% True vs estimated
figure(3), clf
subplot(4,1), hold on
    title("X Position Comparison")
    plot(t,true_xyz(:,1))
    plot(t, estimate_xyz(:,1))
    legend("OptiTrack", "ArUco Marker")
    xlabel("Time (s)"), ylabel("X Position (m)")
subplot(4,2), hold on
    title("Y Position Comparison")
    plot(t,true_xyz(:,2))
    plot(t, estimate_xyz(:,2))
    legend("OptiTrack", "ArUco Marker")
    xlabel("Time (s)"), ylabel("Y Position (m)")
subplot(4,3), hold on
    title("Z Position Comparison")
    plot(t,true_xyz(:,3))
    plot(t, estimate_xyz(:,3))
    legend("OptiTrack", "ArUco Marker")
    xlabel("Time (s)"), ylabel("Z Position (m)")
subplot(4,4), hold on
    plot(t, EucError), legend("Euclidean Error")


% Unique test figures
switch testNo
    case 4
        % if depth test, also show change in euclidean error over z-distance
        figure(4), clf
        title("Error in axes due to change in depth")
        plot(true_xyz(:,3), err_x, 'r')
        plot(true_xyz(:,3), err_y, 'g')
        plot(true_xyz(:,3), err_z, 'b')
        legend("X error", "Y error", "Z error")
        % Important to consider all axes errors... tells us the amount
        % of error it generally has the further away / smaller the marker is

    case 5
        % if xy test, show change in xyz directions
        figure(5), clf
        sgtitle(sprintf("Change in XY Directions (%s)")), hold on
        subplot(2,2,1), hold on
            plot(true_xyz(:,1), true_xyz(:,2))
            plot(estimate_xyz(:,1), estimate_xyz(:,2))
            xlabel("Position (m)"), ylabel("Y Position (m)")
        subplot(2,2,2), hold on
            plot(err_xyz(:,2), true_xyz(:,2))
            xlabel("Y Error")

        subplot(2,2,3), hold on
            plot(err_xyz(:,1), true_xyz(:,1))
            xlabel("X Error")



end

