clc, clear all, close all
%% World transformations - Test 1 Revision 1
load calibrationSession11.mat

origin_t = [0;0;0]; % 3x1 matrix
origin_R = eye(3); % rotation matrix with no rotation


cam_est_t = [-800; 1700; 2420]; % translation only
board_Z = 828.5;

% CALCULATE CAMERA ROTATIONS
cam_est_R = eul2rotm([pi, 0, pi]);

% camera x-axis is inverted (RS)
cam_est_R = cam_est_R * [-1 0 0; 0 1 0; 0 0 1];


% Rotations from origin to camera;
figure(1), clf, hold on
    DisplayAxes("Origin", 'k', origin_t, origin_R), hold on
    DisplayAxes("Camera", 'r', cam_est_t, cam_est_R)
xlabel('X'), ylabel('Y'), zlabel('Z')
legend('x','y','z')




%% FUNCTIONS %%

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
    T = [R, t; 0,0,0,1];

    % gcf, hold on
    for ii = 1:3
        % new_vecs(:,ii) = R * vecs(:,ii) + t;
        new_vecs(:,ii) = T * [vecs(:,ii); 1];
        i = [t(1), new_vecs(1,ii)];
        j = [t(2), new_vecs(2,ii)];
        k = [t(3), new_vecs(3,ii)];

        plot3(i, j, k, 'Color', axesCols(ii));
    end

    plot3(t(1), t(2), t(3),...
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


