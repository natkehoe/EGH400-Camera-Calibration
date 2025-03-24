% Load the calibration session
calibrationData = load('C:\Users\nhkje\git\EGH400-Camera-Calibration\TEST1_CameraCalibration1\calibrationSession11.mat');
 
% Extract camera parameters
cameraParams = calibrationData.calibrationSession.CameraParameters;
 
% Load the new image (that wasn't in the session)
imageFileName = "C:\Users\nhkje\EGH400\Test1Data\imgs\img2025-03-21 18_09_53.864305.png";
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
 
% Convert the rotation matrix to Euler angles (XYZ convention)
eul = rad2deg(rotm2eul(R));  % Euler angles (roll, pitch, yaw)
 
% Convert the rotation matrix to a quaternion
quat = rotm2quat(R);  % Quaternion [qw, qx, qy, qz] (Note: OpenCV quaternion format is different)
 
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

marker.position = P_camera_marker(1:3);
marker.eul = eul(1:3)';
% marker.q.w = quat(1);
% marker.q.x = quat(2);
% marker.q.y = quat(3);
% marker.q.z = quat(4);
marker.q = quat; % [w, x, y, z] format

save camCheckerboardTracking.mat calibrationData imageFileName  marker