function [new_t, new_R, T] = Get2Transform(curr_t, curr_R, frame_t, frame_R)
% GET2TRANSFORM
% [new_t, new_R, T] = Get2Transform(curr_t, curr_R, frame_t, frame_R)
% 
% Get the new transformation from points based on the current pose (curr_t, curr_R) and 
% the frame pose (frame_t, frame_R)
% 
% OUTPUTS: 
% new_t = new translation vector
% new_R = new rotation matrix
% T = new transformation matrix (see GetTransformMat)

    T = GetTransformMat(frame_t, frame_R);

    new_t = T * [curr_t(1:3); 1];
    new_t = new_t(1:3); % remove additional 1 at end - homogeneous to normal
    new_R = curr_R * frame_R;
    new_R = frame_R * curr_R;
end