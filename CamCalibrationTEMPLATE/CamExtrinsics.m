function output = CamExtrinsics(calibrationData, imageFileName)
%% ! RUN VIA MAIN.M
% Sourced from: https://au.mathworks.com/help/vision/ref/estimateextrinsics.html

% Get cam parameters
cameraParams = calibrationData.calibrationSession.CameraParameters;
intrinsics = cameraParams.Intrinsics;

% % LOAD IMAGES
% images = imageDatastore

% Load (new) image to get extrinsics from
image = imread(imageFileName);

% Show raw image
figure(100), imshow(image), title("CamExtrinsics - Raw Input Image")

% Undistort image
[image2, newIntrinsics] = undistortImage(image, intrinsics, OutputView = "full");

% Show undistorted image
figure(101), imshow(image2), title("CamExtrinsics - Undistorted Image")


% Detect points from calibration images
squareSize = calibrationData.calibrationSession.PatternSet.SquareSize; % size of one square on the checkerboard [mm]
% [imagePoints, boardSize] = detectCheckerboardPoints(image); % OLD
[imagePoints, ~] = detectCheckerboardPoints(image2); % detect keypoints of checkerboard calibration image

% Compensate for coordinate system shift
imOrigin = intrinsics.PrincipalPoint - newIntrinsics.PrincipalPoint;
imagePoints = imagePoints + imOrigin; % REQUIRED - now looking at 'undistorted' image

% % Check if checkerboard was detected (OLD METHOD)
% [R,t] = extrinsics(imagePoints, cameraParams.WorldPoints, cameraParams); % [NOT RECOMMENDED]

% --- ESTIMATE EXTRINSICS - NEW ---
camExtrinsics = estimateExtrinsics(imagePoints, cameraParams.WorldPoints, newIntrinsics);

pose = extr2pose(camExtrinsics);
R = pose.R;
t = pose.Translation;

% --- END ESTIMATE EXTRINSICS ---

P_camera = -R * t';
% P_camera = R * t';

P_q = rotm2quat(R);

% Save to output
output.R = R;
output.t = t;
output.marker2camera.x = P_camera(1);
output.marker2camera.y = P_camera(2);
output.marker2camera.z = P_camera(3);
output.marker2camera.qx = P_q(1);
output.marker2camera.qy = P_q(2);
output.marker2camera.qz = P_q(3);
output.marker2camera.qw = P_q(4);

output.imageFileName = imageFileName;

% Display marker2camera transform
fprintf('\n\nmarker2camera transformation:\n')
fprintf('    x: %.2f mm\n', output.marker2camera.x) % Translation
fprintf('    y: %.2f mm\n', output.marker2camera.y)
fprintf('    z: %.2f mm\n', output.marker2camera.z)
fprintf('    qx: %.4f\n', output.marker2camera.qx) % Rotation (quaternion)
fprintf('    qy: %.4f\n', output.marker2camera.qy)
fprintf('    qz: %.4f\n', output.marker2camera.qz)
fprintf('    qw: %.4f\n', output.marker2camera.qw)

end