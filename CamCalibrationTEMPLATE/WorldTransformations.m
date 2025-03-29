function WorldTransformations(savematfile, camCheckerboardTracking, optiMarker)
%% ! RUN FROM MAIN.M


%% COMMON TRANSFORMATIONS %%
inv_XYZ = [-1 0 0; 0 -1 0; 0 0 -1];
inv_X = [-1 0 0; 0 1 0; 0 0 1]; % invert X
inv_Y = [1 0 0; 0 -1 0; 0 0 1];
inv_Z = [1 0 0; 0 1 0; 0 0 -1];

origin_t = [0;0;0]; % 3x1 matrix
origin_R = eye(3); % rotation matrix with no rotation
origin_q = quaternion(origin_R, 'rotmat', 'frame');

% Origin frame transform
origin_T = GetTransformMat(origin_t, origin_R);


%% Camera positioning

cam_est_t = [-800; 1700; 2420]; % translation only ------------------------ EDIT THIS
board_Z = 828.5;

% CALCULATE CAMERA ROTATIONS
% cam_est_R = eul2rotm([pi, 0, pi]);
cam_est_q = quaternion([pi,0,pi], 'euler', 'ZYX', 'frame'); % Convert from euler degrees to quaternion
cam_est_R = rotmat(cam_est_q, 'frame');


% camera x-axis is inverted (RS)
% cam_est_R = cam_est_R * inv_X; % still needs to be done outside of quaternions

% Origin-to-Camera frame transform
origin2cam_T = GetTransformMat(cam_est_t, cam_est_R);

marker2cam_R = cam_est_R * inv_XYZ; % inverse all when referencing from extrinsics to camera intrinsics
% marker2cam_R = cam_est_R; % inverse all when referencing from extrinsics to camera intrinsics

%% --- OPTITRACK SYSTEM --- %%

% TF frame from checkerboard marker to checkerboard (world position)
checkerboardmarker2checkerboard = [0, -0.014, 0, 0, -1.57079633, 1.57079633];
cm2c_t = checkerboardmarker2checkerboard(1:3)'; % cm2c = short for 'checkerboardmarker2checkerboard'
cm2c_q = quaternion(checkerboardmarker2checkerboard(4:6), 'euler', 'ZYX', 'frame'); % <----- is ZYX correct????
cm2c_R = rotmat(cm2c_q, 'frame');

% OptiTracker marker raw data
optiMarkerRaw_t = optiMarker(1:3);
optiMarkerRaw_q = quaternion(optiMarker(4:7)');

optiMarkerRaw_R = rotmat(optiMarkerRaw_q, 'point'); % frame or point? search

% transform to world...
% [optiMarker_t, optiMarker_R, ~] = Get2Transform(optiMarkerRaw_t, optiMarkerRaw_R, cm2c_t, cm2c_R);
optiMarker_t = optiMarkerRaw_t;
optiMarker_R = optiMarkerRaw_R;


%% --- CAMERA SYSTEM --- %%
% Get checkerboard position

camCheckerboardPose = camCheckerboardTracking.marker2camera;

% camCheckerboard_t = camCheckerboardTracking.marker.position; % [mm]
camCheckerboard_t = [camCheckerboardPose.x, camCheckerboardPose.y, camCheckerboardPose.z]'; % [mm]
camCheckerboard_qMat = [camCheckerboardPose.qw, camCheckerboardPose.qx, camCheckerboardPose.qy, camCheckerboardPose.qz]; % [qw, qx, qy, qz]

camCheckerboard_q = quaternion(camCheckerboard_qMat);
camCheckerboard_R = rotmat(camCheckerboard_q, 'point');

% transform to world
[check2world_t, check2world_R] = Get2Transform(camCheckerboard_t, camCheckerboard_R,...
                                                    cam_est_t, marker2cam_R);


%% --- FIGURE --- %%

% Rotations from origin to camera;
figure(1), clf
    grid on, hold on
    DisplayAxes("Origin", 'k', origin_t, origin_R), hold on
    DisplayAxes("Camera", 'r', cam_est_t, cam_est_R)
    % DisplayAxes("LocalOrigin", 'b', loc_origin_t, loc_origin_R)
    % DisplayAxes("Optitrack Marker", 'y', optiMarkerRaw_t, optiMarkerRaw_R)
    DisplayAxes("Optitrack Marker", 'y', optiMarker_t, optiMarker_R)
    DisplayAxes("Camera Marker", 'g', check2world_t, origin_R)
xlabel('X'), ylabel('Y'), zlabel('Z')
xlim([-2000,0]), ylim([0,2000])
legend({'' '' '' 'Origin', ...
        '' '' '' 'Camera', ...
        '' '' '' 'Optitrack Marker', ...
        '' '' '' 'CheckerboardMarker'})


figure(2), clf
[A, map] = imread(camCheckerboardTracking.imageFileName);
imshow(A, map)


%% --- FUNCTIONS --- %%
function [new_t, new_R, T] = Get2Transform(curr_t, curr_R, frame_t, frame_R)
    % get the new transformation from points based on the current pose (curr_t, curr_R) and the frame (t, R)
    % curr_pos = [x,y,z,qw,qx,qy,qz]
    % frame_t = translation vector
    % frame_R = rotation matrix

    T = GetTransformMat(frame_t, frame_R);

    new_t = T * [curr_t(1:3); 1];
    new_t = new_t(1:3); % remove additional 1 at end - homogeneous to normal
    new_R = curr_R * frame_R;
end

function T = GetTransformMat(t, R)
    % curr_pos = [x,y,z,qw,qx,qy,qz]
    % t = translation vector
    % R = rotation matrix

    T = [R, t; 0,0,0,1];
end

function DisplayAxes(name, color, t, R)
    % Using homogenous matrix multiplications...
    % [P_b; 1] = T * [P_a; 1]   where T = [R, t; 0,0,0,1]
    % translate (T) [3x1 vec] and rotate (R) [3x3 mat] from origin point

    axesCols = ['r', 'g', 'b'];

    vecs = [1,0,0;... 
            0,1,0;... 
            0,0,1].*200;    % [x; y; z] axes lines
    
    new_vecs = zeros(4,3);
    new_vecs(4,:) = 1;

    % Transformation matrix... (4x4)
    T = GetTransformMat(t, R);

    % gcf, hold on
    for ii = 1:3
        % new_vecs(:,ii) = R * vecs(:,ii) + t;
        new_vecs(:,ii) = T * [vecs(:,ii); 1];
        i = [t(1), new_vecs(1,ii)];
        j = [t(2), new_vecs(2,ii)];
        k = [t(3), new_vecs(3,ii)];

        plot3(i, j, k, 'Color', axesCols(ii));
    end

    plot3(t(1), t(2), t(3), ...
         'LineStyle', 'none', ...
          'MarkerFaceColor', color, 'MarkerEdgeColor', color, 'Marker','o')
end


% function DisplayAxes(name, color, t, R)
%     % translate (T) [3x1 vec] and rotate (R) [3x3 mat] from origin point
%     vecs = [1,0,0;... 
%             0,1,0;... 
%             0,0,1].*200;    % [x; y; z] axes
% 
%     axesCols = ['r', 'g', 'b'];
% 
%     new_vecs = zeros(3,3);
% 
%     % gcf, hold on
%     for ii = 1:3
%         new_vecs(:,ii) = R * vecs(:,ii) + t;
%         i = [t(1), new_vecs(1,ii)];
%         j = [t(2), new_vecs(2,ii)];
%         k = [t(3), new_vecs(3,ii)];
% 
%         plot3(i, j, k, 'Color', axesCols(ii));
%     end
% 
%     plot3(t(1), t(2), t(3),...
%           'MarkerFaceColor', color, 'MarkerEdgeColor', color, 'Marker','o')
% end

save(savematfile)

end
