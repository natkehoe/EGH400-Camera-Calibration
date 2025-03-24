%% CAMERA CALIBRATION
clc, clear all, close all

%% 1 Get extrinsics
revision = '';
calibrationDataMat = "calibrationSession11.mat"; % filename of camera calibration
calibrationData = load(calibrationDataMat);

imageFileName = "img2025-03-21 18_09_53.864305.png"; % checkerboard image


saveCheckerboardTrackingFileName = ['camCheckerboardTracking', revision, '.mat'];

% Calculate and save checkerboard location to .mat file
    % extrinsics_test(saveCheckerboardTrackingFileName, calibrationData, imageFileName)
    % camCheckerboardTracking = load(saveCheckerboardTrackingFileName);
camCheckerboardTracking = extrinsics_test(calibrationData, imageFileName)


%% Get checkerboard position - OPTITRACKER

% optiMarker raw output:
opti.x = -0.2190161943435669 * 1000; % convert to [mm] ----------------- EDIT THIS
opti.y = 1.783937931060791 * 1000;
opti.z =  0.8732653856277466 * 1000;
opti.ux = -0.0023452339228242636;
opti.uy = -0.00595305347815156;
opti.uz = 0.4090860188007355;
opti.w = -0.9124734401702881;

% Create vectors and extract transforms
optiMarker = [opti.x; ...
              opti.y; ...
              opti.z; ...
              opti.ux; ...
              opti.uy; ...
              opti.uz; ...
              opti.w];

%% 2 Run WorldTransformations.m
saveTransforms = ['worldTransforms', revision, '.mat'];

% calculate and save world transforms
WorldTransformations(saveTransforms, camCheckerboardTracking, optiMarker)
load(saveTransforms)

% Difference in optitracker and marker positions:
diff = optiMarker_t - check2world_t






