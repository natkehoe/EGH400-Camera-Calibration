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
cam_est_q = quaternion([pi,0,0], 'euler', 'ZYX', 'frame'); % Convert from euler degrees to quaternion
cam_est_R = rotmat(cam_est_q, 'frame');


% camera x-axis is inverted (RS)
% cam_est_R = cam_est_R * inv_Z; % makes point go up instead of down

% Origin-to-Camera frame transform
origin2cam_T = GetTransformMat(cam_est_t, cam_est_R);



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
optiMarker_t = optiMarkerRaw_t; % change this?
optiMarker_R = optiMarkerRaw_R;


%% --- CAMERA SYSTEM --- %%
% Get checkerboard position

marker2camera = camCheckerboardTracking.marker2camera;

% camCheckerboard_t = camCheckerboardTracking.marker.position; % [mm]
camCheckerboard_t = [marker2camera.x, marker2camera.y, marker2camera.z]'; % [mm]
camCheckerboard_qMat = [marker2camera.qw, marker2camera.qx, marker2camera.qy, marker2camera.qz]; % [qw, qx, qy, qz]

camCheckerboard_q = quaternion(camCheckerboard_qMat);
camCheckerboard_R = rotmat(camCheckerboard_q, 'point');

% transform to world
[check2world_t, check2world_R] = Get2Transform(camCheckerboard_t, camCheckerboard_R,...
                                                    cam_est_t, cam_est_R);




%% --- FUNCTIONS --- %%






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
