clc; clear all; close all

testNo = 2;
testSelect = {"Old depth",...
            "Static - Central", ...
            "Static - Edge", ...
            "Depth", ...
            "XY", ...
            "Rotations" ...
            };

% add file directories to rosbags with Aruco-Optitrack recordings
testBagFile = {...
    "", ...
    "C:\Users\nhkje\EGH400\Aruco ROSBAG Recordings\20250424-static\20250424-static_0.db3", ...
    "", ...
    "", ...
    "C:\Users\nhkje\EGH400\Aruco ROSBAG Recordings\20250424-xy\20250424-xy_0.db3", ...
    "" ...
};

% load rosbag
    % https://au.mathworks.com/help/ros/ref/ros2bagreader.readmessages.html
    % https://au.mathworks.com/help/ros/ref/readmessages.html?searchHighlight=readmessages&s_tid=srchtitle_support_results_1_readmessages


bagReader = ros2bagreader(testBagFile{testNo});


timeStart = bagReader.StartTime; % bag start time
optiTopic = select(bagReader, "Topic", "/vrpn_client_node/tronpi21/pose");
optiMsgs = readMessages(optiTopic);

arucoTopic = select(bagReader, "Topic", "/rover_centre");
arucoMsgs = readMessages(arucoTopic);

% get messages that have the same timestamp (not doing; not possible)



%% "Downsample" OptiTrack data
optiMsgs_time = optiTopic.MessageList.Time - timeStart;
optiTimeMasked = ones(size(optiMsgs_time));
arucoMsgs_time = arucoTopic.MessageList.Time - timeStart;

optiMsgs_time2 = nan(size(arucoMsgs_time)); % new time measurement to compare against arucoMsgs
optiMsgs_select = [];

% find nearest optiMsgs_time's to arucoMsgs_time (no doubling)
for i = 1:length(arucoMsgs_time)
    % Find OptiTrack timestamp closest to arucoMsgs
    [~, I] = min(abs(optiMsgs_time.*optiTimeMasked - arucoMsgs_time(i)));
    
    % add to t
    optiMsgs_time2(i) = optiMsgs_time(I);
    optiMsgs_select(i) = I;

    % if duplicate values, remove from search
    % optiTimeMasked(I) = 0;
end


% get optiTrack new data
figure(100), clf, title("Time downsampling difference"), hold on
plot(arucoMsgs_time, arucoMsgs_time - optiMsgs_time2)
xlabel("Time (s)"), ylabel("Time difference (s)")

%% Extract pose

% Select optitrack points based on sampled time
% get optitrack BASE LINK transform
% true_xyz = [cellfun(@(msg) msg.pose.position.x, optiMsgs), ...
%             cellfun(@(msg) msg.pose.position.y, optiMsgs), ...
%             cellfun(@(msg) msg.pose.position.z, optiMsgs)];
% % get optitrack BASE LINK rotation matrix
% true_quat = [cellfun(@(msg) msg.pose.orientation.x, optiMsgs), ...
%             cellfun(@(msg) msg.pose.orientation.y, optiMsgs), ...
%             cellfun(@(msg) msg.pose.orientation.z, optiMsgs), ...
%             cellfun(@(msg) msg.pose.orientation.w, optiMsgs)];
% % select only the ones we want
% true_xyz = true_xyz(optiMsgs_select,:);
% true_quat = true_quat(optiMsgs_select,:);
trueData = ConvertMsgsToMat(optiMsgs(optiMsgs_select));

% get /rover_centre output
% estimate_xyz = [cellfun(@(msg) msg.pose.position.x, arucoMsgs), ...
%                 cellfun(@(msg) msg.pose.position.y, arucoMsgs), ...
%                 cellfun(@(msg) msg.pose.position.z, arucoMsgs)];
% estimate_quat = [cellfun(@(msg) msg.pose.orientation.x, arucoMsgs), ...
%                 cellfun(@(msg) msg.pose.orientation.y, arucoMsgs), ...
%                 cellfun(@(msg) msg.pose.orientation.z, arucoMsgs), ...
%                 cellfun(@(msg) msg.pose.orientation.w, arucoMsgs)];
estData = ConvertMsgsToMat(arucoMsgs);


%% Calculate error
% Calculate xyz error
err_x = estData.xyzMat(:,1) - trueData.xyzMat(:,1);
err_y = estData.xyzMat(:,2) - trueData.xyzMat(:,2);
err_z = estData.xyzMat(:,3) - trueData.xyzMat(:,3);

EucError = sqrt(err_x.^2 + err_y.^2 + err_z.^2);


% Calculate rpy error
err_roll = abs(trueData.rpyMat(:,1) - estData.rpyMat(:,1));
err_pitch = abs(trueData.rpyMat(:,2) - estData.rpyMat(:,2));
err_yaw = abs(trueData.rpyMat(:,3) - estData.rpyMat(:,3));

%% Find mean error
mean_xyz = mean([err_x, err_y, err_z])
mean_rpy = mean([err_roll, err_pitch, err_yaw])

% truexyzCov = ComputeCovarianceMat(trueData.xyzMat)
% estxyzCov = ComputeCovarianceMat(estData.xyzMat)

cov_xyz = ComputeCovarianceMat([err_x, err_y, err_z]) % [metres]
cov_rpy = ComputeCovarianceMat([err_roll, err_pitch, err_yaw]) % [radians]

%% Get 

% subtract mean
estData.xyzMat = estData.xyzMat - mean_xyz;

test = ConvertMsgsToMat(optiMsgs(optiMsgs_select))

% FIGURES
% Euclidean Distance Error of Localisation System - [test name]
figure(1), clf
title(sprintf("Euclidean Distance Error of Localisation System (%s)", testSelect{testNo}))
grid on, hold on
plot(optiMsgs_time2, EucError)
xlabel("Time (s)"), ylabel("Euclidean Error (m)")

    % if roll, show roll-pitch-yaw change over time

% Individual axes errors 
figure(2), clf
title(sprintf("Error in Individual Axes (%s)", testSelect{testNo}))
grid on, hold on
plot(arucoMsgs_time, err_x, 'r')
plot(arucoMsgs_time, err_y, 'g')
plot(arucoMsgs_time, err_z, 'b')
legend("X error", "Y error", "Z error")
xlabel("Time (s)"), ylabel("Euclidean Error (m)")

% True vs estimated
figure(3), clf
subplot(4,1,1), hold on
    title("X Position Comparison")
    plot(arucoMsgs_time,trueData.xyzMat(:,1))
    plot(arucoMsgs_time, estData.xyzMat(:,1))
    legend("OptiTrack", "ArUco Marker")
    xlabel("Time (s)"), ylabel("X Position (m)")
subplot(4,1,2), hold on
    title("Y Position Comparison")
    plot(arucoMsgs_time,trueData.xyzMat(:,2))
    plot(arucoMsgs_time, estData.xyzMat(:,2))
    legend("OptiTrack", "ArUco Marker")
    xlabel("Time (s)"), ylabel("Y Position (m)")
subplot(4,1,3), hold on
    title("Z Position Comparison")
    plot(arucoMsgs_time,trueData.xyzMat(:,3))
    plot(arucoMsgs_time, estData.xyzMat(:,3))
    legend("OptiTrack", "ArUco Marker")
    xlabel("Time (s)"), ylabel("Z Position (m)")
subplot(4,1,4), hold on
    plot(arucoMsgs_time, EucError), legend("Euclidean Error")


% Unique test figures
switch testNo
    case 4
        % if depth test, also show change in euclidean error over z-distance
        figure(4), clf
        title("Error in axes due to change in depth")
        plot(trueData.xyzMat(:,3), err_x, 'r')
        plot(trueData.xyzMat(:,3), err_y, 'g')
        plot(trueData.xyzMat(:,3), err_z, 'b')
        legend("X error", "Y error", "Z error")
        % Important to consider all axes errors... tells us the amount
        % of error it generally has the further away / smaller the marker is

    case 5
        % if xy test, show change in xyz directions
        figure(5), clf
        sgtitle(sprintf("Change in XY Directions (%s)")), hold on
        subplot(2,2,1), hold on
            plot(trueData.xyzMat(:,1), trueData.xyzMat(:,2))
            plot(estData.xyzMat(:,1), estData.xyzMat(:,2))
            xlabel("Position (m)"), ylabel("Y Position (m)")
        subplot(2,2,2), hold on
            plot(err_xyz(:,2), trueData.xyzMat(:,2))
            xlabel("Y Error")

        subplot(2,2,3), hold on
            plot(err_xyz(:,1), trueData.xyzMat(:,1))
            xlabel("X Error")



end

function x = ConvertMsgsToMat(rosMsgs)
    x.xyzMat = [cellfun(@(msg) msg.pose.position.x, rosMsgs), ...
                cellfun(@(msg) msg.pose.position.y, rosMsgs), ...
                cellfun(@(msg) msg.pose.position.z, rosMsgs)];
    x.quatMat = [cellfun(@(msg) msg.pose.orientation.x, rosMsgs), ...
                 cellfun(@(msg) msg.pose.orientation.y, rosMsgs), ...
                 cellfun(@(msg) msg.pose.orientation.z, rosMsgs), ...
                 cellfun(@(msg) msg.pose.orientation.w, rosMsgs)];
    x.rpyMat = rotvec(quaternion(x.quatMat));


end

function covarianceMat = ComputeCovarianceMat(x)
    % Params should be a matrix of format (lengthOverTime) x (sumOfVariables).

    xWidth = width(x);
    % xLength = length(x);
    % 
    % xMean = mean(x, 1); % find mean for each column

    xVars = var(x);

    covarianceMat = eye(xWidth) .* xVars;
    
    
    
    
end