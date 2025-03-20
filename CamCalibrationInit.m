%% INIT - an outline of the camera calibration setup 
%% determining how large the ARUCO and CHECKERBOARD markers will need to be for effective testing
% Author: Natalie Kehoe

poseCam = [0,0,2800];
poseOrigin = [0,0,0];


h1 = 2.8; % ground to cam - initial guess = 2.8 [m]
h2 = 0; % ground to 


%% Camera specs - Intel RealSense L515
% https://www.intelrealsense.com/lidar-camera-l515/
cam = struct;
cam.Pose = poseCam;
cam.FoV = deg2rad([70, 43]); % [horizontal x vertical] [rad]
cam.Res = [1920, 1080];


%% Save data
save init.mat
