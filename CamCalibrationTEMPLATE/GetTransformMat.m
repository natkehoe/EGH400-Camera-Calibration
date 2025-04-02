function T = GetTransformMat(t, R)
% GETTRANSFORMMAT Create a 4x4 Transform Matrix base on translation vector and rotation
% matrix
% 
% t = translation vector [x, y, z]
% R = rotation matrix [r1, r2, r3; r4, r5, r6; r7, r8, r9]
% 
% Output: T = [r1, r2, r3, x; 
%              r4, r5, r6, y;
%              r7, r8, r9, z;
%               0,  0,  0, 1 ]
    T = [R, t; 0,0,0,1];
end