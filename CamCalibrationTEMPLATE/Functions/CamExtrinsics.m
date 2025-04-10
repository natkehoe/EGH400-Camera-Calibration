function output = CamExtrinsics(calibrationData, imageFileName)
%% ! RUN VIA MAIN.M
% Sourced from: https://au.mathworks.com/help/vision/ref/estimateextrinsics.html

detectOption = 2.1; % [1 = use original image; 2 = undistort image BEFORE detecting points; 3 = undistort detected points AFTER.] 
    % 1 is most reliable, due to PatternExtrinsics in distorted view.

% Get cam parameters
cameraParams = calibrationData.calibrationSession.CameraParameters;
% cameraParams = cameraParameters(toStruct(calibrationData.calibrationSession.CameraParameters)); % FORMAT USED
intrinsics = cameraParams.Intrinsics;

worldPoints = cameraParams.WorldPoints;

image = imread(imageFileName);

% --- DETECT POINTS --- %
switch detectOption
    case 1
        % Use original image
        [imagePoints, ~] = detectCheckerboardPoints(image); % detect keypoints of checkerboard calibration image


        % if imagePoints not detectable, remove points for both imagePoints
        % and WorldPoints
            if any(isnan(imagePoints), 'all')
                warning("NOT ALL IMAGE POINTS DETECTED: %s", imageFileName);
                p = find(isnan(imagePoints(:,1)));

                % Remove these points for exctrinsics estimation purposes
                imagePoints(p,:) = [];
                worldPoints(p,:) = [];

            end


        camExtrinsics = estimateExtrinsics(imagePoints, worldPoints, intrinsics);

    case 2
        % undistort image BEOFRE detecting points
        
        % Undistort image
        [image2, newIntrinsics] = undistortImage(image, intrinsics, OutputView = "same"); % "SAME" has least difference error
        
        % Show undistorted image
        figure(101), imshow(image2), title("CamExtrinsics - Undistorted Image (2.0)")
        [imagePoints, ~] = detectCheckerboardPoints(image2); % detect keypoints of checkerboard calibration image

        % Compensate for coordinate system shift
        % imOrigin = intrinsics.PrincipalPoint - newIntrinsics.PrincipalPoint;
        % imagePoints = imagePoints + imOrigin; % REQUIRED - now looking at 'undistorted' image
        
        camExtrinsics = estimateExtrinsics(imagePoints, worldPoints, newIntrinsics);
    case 2.1
        % undistort image BEOFRE detecting points - EDITS
        
        % Undistort image
        % [image2, newIntrinsics] = undistortImage(image, intrinsics, OutputView = "same"); % "SAME" has least difference error
        [image2, newIntrinsics] = undistortImage(image, cameraParams); % refer to (https://au.mathworks.com/help/vision/ref/undistortimage.html)


        [imagePoints, ~] = detectCheckerboardPoints(image2); % detect keypoints of checkerboard calibration image

        % if imagePoints not detectable, remove points for both imagePoints
        % and WorldPoints
            if any(isnan(imagePoints), 'all')
                warning("NOT ALL IMAGE POINTS DETECTED: %s", imageFileName);
                p = find(isnan(imagePoints(:,1)));

                % Remove these points for exctrinsics estimation purposes
                imagePoints(p,:) = [];
                worldPoints(p,:) = [];

            end


        % % Compensate for coordinate system shift
        imOrigin = intrinsics.PrincipalPoint - newIntrinsics.PrincipalPoint;
        imagePoints = imagePoints + imOrigin; % REQUIRED - now looking at 'undistorted' image
        
        camExtrinsics = estimateExtrinsics(imagePoints, worldPoints, newIntrinsics);

    case 3
        % undistort points AFTER detection (https://au.mathworks.com/help/vision/ref/undistortpoints.html)
        % image3 = image;

        % detect points on raw image
        points = detectCheckerboardPoints(image);

        % undistort points
        undistortedPoints = undistortPoints(points, intrinsics);

        % undistort image
        [image3, newIntrinsics] = undistortImage(image, cameraParams);

        % translate undistorted points
        imOrigin = intrinsics.PrincipalPoint - newIntrinsics.PrincipalPoint;
        undistortedPoints = [undistortedPoints(:,1) - imOrigin(1), ...
                             undistortedPoints(:,2) - imOrigin(2)];

        % solve and estimate Extrinsics
        camExtrinsics = estimateExtrinsics(undistortedPoints, worldPoints, newIntrinsics);
end


% % Load (new) image to get extrinsics from
% image = imread(imageFileName);

% Show raw image



% % Detect points from calibration images
% squareSize = calibrationData.calibrationSession.PatternSet.SquareSize; % size of one square on the checkerboard [mm]
% [imagePoints, boardSize] = detectCheckerboardPoints(image); % OLD
% % [imagePoints, ~] = detectCheckerboardPoints(image2); % detect keypoints of checkerboard calibration image
% [imagePoints, ~] = detectCheckerboardPoints(image); % detect keypoints of checkerboard calibration image

% Compensate for coordinate system shift
imOrigin = intrinsics.PrincipalPoint - newIntrinsics.PrincipalPoint;
imagePoints = imagePoints + imOrigin; % REQUIRED - now looking at 'undistorted' image

% % Check if checkerboard was detected (OLD METHOD)
% [R,t] = extrinsics(imagePoints, worldPoints, cameraParams); % [NOT RECOMMENDED]

% --- ESTIMATE EXTRINSICS - NEW ---
% camExtrinsics = estimateExtrinsics(imagePoints, worldPoints, newIntrinsics);
% camExtrinsics = estimateExtrinsics(imagePoints, worldPoints, intrinsics);
% 
% pose = extr2pose(camExtrinsics);
% R = pose.R;
% t = pose.Translation;

R = camExtrinsics.R;
t = camExtrinsics.Translation;

% --- END ESTIMATE EXTRINSICS ---

P_camera = -R * t';
% P_camera = R * t';
% K = cameraParams.K; % <-- not the solution but tried it.
% P_camera = K * (-R * t');

P_q = rotm2quat(R);

% Save to output
output.R = R; % <--
output.t = t; % <--
output.marker2camera.x = P_camera(1);
output.marker2camera.y = P_camera(2);
output.marker2camera.z = P_camera(3);
output.marker2camera.qx = P_q(1);
output.marker2camera.qy = P_q(2);
output.marker2camera.qz = P_q(3);
output.marker2camera.qw = P_q(4);

output.imageFileName = imageFileName;

% % Display marker2camera transform
% fprintf('\n\nmarker2camera transformation:\n')
% fprintf('    x: %.2f mm\n', output.marker2camera.x) % Translation
% fprintf('    y: %.2f mm\n', output.marker2camera.y)
% fprintf('    z: %.2f mm\n', output.marker2camera.z)
% fprintf('    qx: %.4f\n', output.marker2camera.qx) % Rotation (quaternion)
% fprintf('    qy: %.4f\n', output.marker2camera.qy)
% fprintf('    qz: %.4f\n', output.marker2camera.qz)
% fprintf('    qw: %.4f\n', output.marker2camera.qw)

%% SHOW IMAGES
        
figure(100), imshow(image), hold on
    title("CamExtrinsics - Raw Input Image")
    subtitle(imageFileName)

figure(102), imshow(image2), hold on
    title("CamExtrinsics - Undistorted Image (2.1)")
    subtitle(imageFileName)

end