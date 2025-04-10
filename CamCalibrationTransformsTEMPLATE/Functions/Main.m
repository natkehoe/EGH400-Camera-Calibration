%% CAMERA CALIBRATION
clc, clear all, close all

addpath("./Functions/")

%% 1 Get calibration intrinsics and extrinsics
revision = '05';
calibrationDataMat = "calibrationSession11.mat"; % filename of camera calibration
calibSession = load(calibrationDataMat);

printON = 1;

imageNo = [1:40, 42];
% imageNo = 23;


%% Analyse all images
distortionErrors = []; % track errors of distortion across image [xpos, ypos, dEuc]
for i = imageNo


%% Load image
imageFileNames = ls("./CalibrationImages"); 
imageFileNames = "CalibrationImages/" + imageFileNames(3:end, :); % get filenames; ignore {'.', '..'}

% Double-check this is the correct am. files; this should be same as calibSession
% collected images
assert(size(imageFileNames, 1) == calibSession.calibrationSession.CameraParameters.NumPatterns)

% --- SELECT IMAGE HERE --- %
% imageFileName = "img2025-03-21 18_09_53.864305.png   "; % checkerboard image
imageFileName = imageFileNames(i); % checkerboard image


saveCheckerboardTrackingFileName = ['camCheckerboardTracking', revision, '.mat'];

% Calculate and save checkerboard location to .mat file
    % extrinsics_test(saveCheckerboardTrackingFileName, calibrationData, imageFileName)
    % camCheckerboardTracking = load(saveCheckerboardTrackingFileName);
% camCheckerboardTracking = CamExtrinsics(calibSession, imageFileName);
CamExtrinsics(saveCheckerboardTrackingFileName, calibSession, imageFileName);


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

% % Difference in optitracker and marker positions:
% diff1 = optiMarker_t - check2world_t

%% Comparison to cam extrinsics and world transformations
% ORIGIN = origin_x
% CAMERA = cam_est_x <--- TO BE REPLACED.
% OPTITRACK MARKER = optiMarker_x
% CAMERA MERKER = check2world_x

% Throw error if extrinsics not in units [mm]
assert(calibSession.calibrationSession.CameraParameters.WorldUnits == "millimeters", 'Extrinsics not in mm');


% patternExtrinsicsInv_t = patternExtrinsicsRaw_t .* -1; %% invert patternExtrinsics
% patternExtrinsicsInv_t = patternExtrinsicsRaw_t;
=======
% --- GET PATTERN EXTRINSICS --- %
% (based on which image selected) - RAW DISTORTED POINTS
patternExtrinsicsRaw_t = calibSession.calibrationSession.CameraParameters.PatternExtrinsics(i).Translation';
patternExtrinsicsRaw_R = calibSession.calibrationSession.CameraParameters.PatternExtrinsics(i).R;

% UNDISTORT patternExtrinsics


% Get pattern extrinsics to world frame (check2world)
[patternExtrinsicsTransformed_t, patternExtrinsicsTransformed_R, ~] ...
    = Get2Transform(patternExtrinsicsRaw_t, patternExtrinsicsRaw_R, cam_est_t, cam_est_R);

% --- COMPARE --- %
diff2 = patternExtrinsicsTransformed_t - check2world_t;

fprintf("IMAGE %d:\n", i)
if any(diff2)
    dEuc2 = sqrt(sum(diff2.^2)); % Euclidean error
    % fprintf("There is %.4f mm difference between real and estimated check2world points.\n", dEuc2)
    fprintf("Difference between real and est check2world:   %.4f mm\n", dEuc2)

    % diff3 = abs(patternExtrinsicsRaw_t - camCheckerboard_t);
    % diff3 = abs(patternExtrinsicsRaw_t - camCheckerboardTracking.t');
    % dEuc3 = sqrt(sum(diff3.^2)); 
    % fprintf("Difference between real and est marker2camera: %.4f mm\n", dEuc3)
else

    fprintf("Real and estimated check2world points are identical.")
end

%% --- FIGURE --- %%

% Rotations from origin to camera;
f1 = figure(1); clf, hold on
    f1.Position = [200 200 560 420];
    % ax1 = gca(f1);
    % ax1.CameraPosition = [5000,-15000,6000];
    grid on
    DisplayAxes("Origin", 'k', origin_t, origin_R), hold on
    DisplayAxes("Camera", 'r', cam_est_t, cam_est_R)
    DisplayAxes("Optitrack Marker", 'y', optiMarker_t, optiMarker_R)
    % DisplayAxes("Camera Marker", 'g', check2world_t, origin_R)
    DisplayAxes("Camera Marker", 'g', check2world_t, check2world_R)
    DisplayAxes("Camera Extrinsics", 'b', patternExtrinsicsTransformed_t, patternExtrinsicsTransformed_R)
    % DisplayAxes("Raw marker2camera Pose", 'r', camCheckerboardTracking.t', camCheckerboardTracking.R)
    % DisplayAxes("Raw marker2camera Pose", 'magenta', camCheckerboardTracking.t' .* -1, camCheckerboardTracking.R .* -1)
    % DisplayAxes("Raw marker2camera Pose", 'cyan', camCheckerboardTracking.t' .* -1, -camCheckerboardTracking.R)

    title(sprintf("Image %d", i))
    subtitle(sprintf("Error: %.4f mm", dEuc2))
    xlabel('X'), ylabel('Y'), zlabel('Z')
    % xlim([-2000,0]), ylim([0,2000])
    % zlim([0,2000])
    legend({...
            '' '' '' 'Origin', ...
            '' '' '' 'Camera', ...
            '' '' '' 'Optitrack Marker', ...
            '' '' '' 'CheckerboardMarker', ...
            '' '' '' 'Camera Extrinsics'}, "Location","southwest")


f2 = figure(2); clf, hold on
    f2.Position = [760 200 560 420];
    ax2 = gca(f2);
    ax2.CameraPosition = [5000,-15000,6000];
    grid on
    DisplayAxes("Camera Marker", 'g', check2world_t, check2world_R)
    DisplayAxes("Camera Extrinsics", 'b', patternExtrinsicsTransformed_t, patternExtrinsicsTransformed_R)

    title(sprintf("Image %d - Zoomed", i))
    subtitle(sprintf("Error: %.4f mm", dEuc2))
xlabel('X'), ylabel('Y'), zlabel('Z')
legend({...
        '' '' '' 'CheckerboardMarker', ...
        '' '' '' 'Camera Extrinsics', ...
        '' '' '' 'Raw marker2camera Pose'}, "Location","southwest")


%% checks
RPY_pE = rotmat2vec3d(patternExtrinsicsTransformed_R);
RPY_check2world = rotmat2vec3d(check2world_R);
fprintf("\n\nRPY - PatternExtrinsics: [%.2f, %.2f, %.2f]\n", ...
    RPY_pE(1), RPY_pE(2), RPY_pE(3))
fprintf("RPY - marker2camera: [%.2f, %.2f, %.2f]\n", ...
    RPY_check2world(1), RPY_check2world(2), RPY_check2world(3))


%% Get Distortion Error
distortionErrors(end+1, 1:3) = [patternExtrinsicsTransformed_t(1), patternExtrinsicsTransformed_t(2), dEuc2];






end
%% Figure 3
f3 = figure(3); clf
    f3.Position = [418 458 822 420];
    title("Distortion errors")
    subtitle(sprintf("Mean error: %.4f mm", mean(distortionErrors(:,3))))
    xlabel("X"), ylabel("Y")
    ylabel("")

    grid on, hold on
    scatter(distortionErrors(:,1), distortionErrors(:,2), ...
        [], distortionErrors(:,3), 'filled')
    colormap("winter")
    c = colorbar;
    c.Label.String = "Euclidean Error (mm)";

distortionErrors


%% PRINT
if printON
    printFolderName = "Figures/";
    print(f1, '-dpng', sprintf("%sf1_image%d", printFolderName, i))
    print(f2, '-dpng', sprintf("%sf2_image%d", printFolderName, i))
    print(f3, '-dpng', sprintf("%sf3_image%d", printFolderName, i))
end

