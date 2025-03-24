clc, clear all, close all

%% ROS2 BAG READER
% Try 1 at getting messages from rosbag2.

% Get bag location
if (~exist('bagReader', 'var'))
    path = "C:\Users\nhkje\EGH400\Rosbag conversion\ros2_bag_dir\ros2_bag_dir.db3";
    bag = ros2bagreader(path);
    % disp("True")
end

% read messages
msgs = readMessages(bag);

%% Get Optitracker pose
optiTracker_poses = nan(7,0);

for i = 1:length(msgs)
    if (msgs{i}.MessageType == "geometry_msgs/PoseStamped")
        % pose = msgs{i}.pose.position;
        optiTracker_poses(:,end+1) = [ ...
            msgs{i}.pose.position.x; ...
            msgs{i}.pose.position.y; ...
            msgs{i}.pose.position.z; ...
            msgs{i}.pose.orientation.x; ...
            msgs{i}.pose.orientation.y; ...
            msgs{i}.pose.orientation.z; ...
            msgs{i}.pose.orientation.w ...
            ];
    end

end