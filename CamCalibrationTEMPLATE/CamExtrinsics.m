function output = CamExtrinsics(calibrationData, imageFilename)
%% ! RUN VIA MAIN.M
% Sourced from: https://au.mathworks.com/help/vision/ref/estimateextrinsics.html

% Get cam parameters
cameraParams = calibrationData.calibrationSession.CameraParameters;

% Load image
image = imread(imageFilename);

% Detect points
squareSize = calibrationData.calibrationSession.PatternSet.SquareSize; % size of one square on the checkerboard [mm]
[imagePoints, boardSize] = detectCheckerboardPoints(image);

% Check if checkerboard was detected
[R,t] = extrinsics(imagePoints, cameraParams.WorldPoints, cameraParams);

P_camera = -R * t';

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