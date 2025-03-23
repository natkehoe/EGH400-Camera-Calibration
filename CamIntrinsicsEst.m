clc, clear all
%% Estimation of Camera Intrinsics

% manual estimation of camera intrinsics. Used to estimate initial
% intrinsics for camera calibration.
% OUTPUT:
    % camIntrinsics = [ fx 0 0; s fy 0; cx cy 1 ]

%% --- INITIAL VARIABLES --- %%
f = 1.88; % focal length [mm]
sensorFormat = 2.73; % approx 1/6"
sensorAspectRatio = [16, 9]; % 16:9
pixels = [1920, 1080];
FoV = deg2rad([69, 42]); % [horizontal, vertical]

s = 0; % skew (estimated)


%% --- CALCULATIONS --- %%

%% CALCULATE PIXEL SCALING FACTOR (kx, ky) [pixel/mm]
% sensorSize (x, y) [mm] --> x = h*cos(theta)
sensorSize = sensorFormat .* [cos(atan( sensorAspectRatio(2)/sensorAspectRatio(1) )), ...
                              sin(atan( sensorAspectRatio(2)/sensorAspectRatio(1) ))];

% scaling factor
kx = pixels(1) / sensorSize(1); % [
ky = pixels(2) / sensorSize(2);


%% CALCULATE FOCAL LENGTHS (fx, fy)
fx = f * kx;
fy = f * ky;


%% IMAGE CENTRE
cx = pixels(1)/2;
cy = pixels(2)/2;

%% INTRINSICS MATRIX
camIntrinsics = [fx 0 0; s fy 0; cx cy 1];


