% Load the calibration session
calibrationData = load('UPDATED_Session.mat');
% Extract camera parameters
cameraParams = calibrationData.calibrationSession.CameraParameters;
% Load the new image (that wasn't in the session)
%imageFileName = 'secondextrinsic1.png';
imageFileName = 'POSE_extrinsics1.png';
newImage = imread(imageFileName);
% Detect the checkerboard points (adjust the square size and pattern size accordingly)
squareSize = 29;  % Set the size of a square in the checkerboard (in millimeters)

% Check if checkerboard was detected
if isempty(imagePoints)
    error('No checkerboard detected in the image.');
end

imageSize = [size(newImage,1) size(newImage,2)];
intrinsics = cameraParams.Intrinsics;
[im,newIntrinsics] = undistortImage(newImage,intrinsics,OutputView="full");
[imagePoints,boardSize] = detectCheckerboardPoints(im);
worldPoints = cameraParams.WorldPoints;
newOrigin = intrinsics.PrincipalPoint - newIntrinsics.PrincipalPoint;
imagePoints = imagePoints+newOrigin;
camExtrinsics = estimateExtrinsics(imagePoints,worldPoints,newIntrinsics);

inv(camExtrinsics.A)

camPose = extr2pose(camExtrinsics);
figure
plotCamera(AbsolutePose=camPose,Size=20);
hold on
pcshow([worldPoints,zeros(size(worldPoints,1),1)], ...
  VerticalAxisDir="down",MarkerSize=40);


eul = rad2deg(rotm2eul(camPose.R))  % Euler angles (roll, pitch, yaw)
camPose.Translation
quat = rotm2quat(camPose.R);  % Quaternion [qx, qy, qz, qw] (Note: OpenCV quaternion format is different)
