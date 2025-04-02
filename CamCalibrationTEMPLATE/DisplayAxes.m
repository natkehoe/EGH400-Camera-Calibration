function h = DisplayAxes(name, color, t, R)
% DISPLAYAXES Displays object transformations in a predetermined figure
% (number 'fig').
% 
% INPUTS:
%   fig = (int) fig number
%   name = (str) object name
%   color = (char) colour to be used (see plot3)
%   t = translational vector [x; y; z] (3x1)
%   R = rotation matrix (3x3)
%
% OUTPUT: 
%   handles of the 3D ploted axes.
%
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