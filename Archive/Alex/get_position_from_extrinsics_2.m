%% Get position from extrinsics
% Author: Alexander Allan

addpath("./Archive/Alex/")
addpath("./Archive/Alex/Updated_camera_session")

% Load the calibration session
calibrationData = load('UPDATED_Session.mat');
%%
% Extract camera parameters
cameraParams = calibrationData.calibrationSession.CameraParameters;

% Load the new image (that wasn't in the session)
imageFileName = 'secondextrinsic1.png';
newImage = imread(imageFileName);

% Detect the checkerboard points (adjust the square size and pattern size accordingly)
squareSize = 29;  % Set the size of a square in the checkerboard (in millimeters)
[imagePoints, boardSize] = detectCheckerboardPoints(newImage);

% Check if checkerboard was detected
if isempty(imagePoints)
    error('No checkerboard detected in the image.');
end
% Estimate the extrinsics using the new image and the known camera parameters
[R, t] = extrinsics(imagePoints, cameraParams.WorldPoints, cameraParams);
% Compute the camera's position relative to the marker (marker-centric)
P_camera_marker = -R * t';

T = rigidtform3d(R,P_camera_marker);

figure
plotCamera(AbsolutePose=T,Size=20);
hold on
pcshow([worldPoints,zeros(size(worldPoints,1),1)], ...
  VerticalAxisDir="down",MarkerSize=40);


% Convert the rotation matrix to Euler angles (XYZ convention)
eul2 = rad2deg(rotm2eul(R));  % Euler angles (roll, pitch, yaw)
% Convert the rotation matrix to a quaternion
quat = rotm2quat(R);  % Quaternion [qx, qy, qz, qw] (Note: OpenCV quaternion format is different)
% Display the camera's position relative to the marker
fprintf('Camera position relative to marker (marker-centric frame):\n');
fprintf('X: %.2f mm\n', P_camera_marker(1));
fprintf('Y: %.2f mm\n', P_camera_marker(2));
fprintf('Z: %.2f mm\n', P_camera_marker(3));
% Display the Euler angles (roll, pitch, yaw)
fprintf('Camera orientation (Euler angles in degrees):\n');
fprintf('Roll (X): %.4f deg\n', eul(1));
fprintf('Pitch (Y): %.4f deg\n', eul(2));
fprintf('Yaw (Z): %.4f deg\n', eul(3));
% Display the quaternion
fprintf('Camera orientation (Quaternion):\n');
fprintf('qw: %.4f, qx: %.4f, qy: %.4f, qz: %.4f\n', quat(1), quat(2), quat(3), quat(4));