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
camCheckerboardTracking = CamExtrinsics(calibrationData, imageFileName)


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

% calculate and save world transforms in .mat specified in 'saveTransforms'
WorldTransformations(saveTransforms, camCheckerboardTracking, optiMarker)
load(saveTransforms)

% Difference in optitracker and marker positions:
diff = optiMarker_t - check2world_t

%% Comparison to cam extrinsics and world transformations
% ORIGIN = origin_x
% CAMERA = cam_est_x <--- TO BE REPLACED.
% OPTITRACK MARKER = optiMarker_x
% CAMERA MERKER = check2world_x

% Throw error if extrinsics not in units [mm]
assert(calibrationData.calibrationSession.CameraParameters.WorldUnits == "millimeters", 'Extrinsics not in mm');

% Get raw patternExtrinsics (marker2camera)
patternExtrinsicsRaw_t = calibrationData.calibrationSession.CameraParameters.PatternExtrinsics(1,1).Translation';
patternExtrinsicsRaw_R = calibrationData.calibrationSession.CameraParameters.PatternExtrinsics(1,1).R;

% patternExtrinsicsInv_t = patternExtrinsicsRaw_t .* -1; %% invert patternExtrinsics
patternExtrinsicsInv_t = patternExtrinsicsRaw_t;

% Get pattern extrinsics to world frame (check2world)
[patternExtrinsicsTransformed_t, patternExtrinsicsTransformed_R, ~] ...
    = Get2Transform(patternExtrinsicsInv_t, patternExtrinsicsRaw_R, cam_est_t, cam_est_R);

diff = patternExtrinsicsTransformed_t - check2world_t;

if any(diff)
    dEuclidean = sqrt(sum(diff.^2));
    fprintf("There is %.4f mm difference between real and estimated check2world points.\n", dEuclidean)
else

    fprintf("Real and estimated check2world points are identical.")
end

%% --- FIGURE --- %%

% Rotations from origin to camera;
figure(1), clf
    grid on, hold on
    DisplayAxes("Origin", 'k', origin_t, origin_R), hold on
    DisplayAxes("Camera", 'r', cam_est_t, cam_est_R)
    DisplayAxes("Optitrack Marker", 'y', optiMarker_t, optiMarker_R)
    DisplayAxes("Camera Marker", 'g', check2world_t, origin_R)
    DisplayAxes("Camera Extrinsics", 'b', patternExtrinsicsTransformed_t, patternExtrinsicsTransformed_R)
xlabel('X'), ylabel('Y'), zlabel('Z')
% xlim([-2000,0]), ylim([0,2000])
legend({'' '' '' 'Origin', ...
        '' '' '' 'Camera', ...
        '' '' '' 'Optitrack Marker', ...
        '' '' '' 'CheckerboardMarker', ...
        '' '' '' 'Camera Extrinsics'}, "Location","southwest")


% figure(2), clf
% [A, map] = imread(camCheckerboardTracking.imageFileName);
% imshow(A, map)









