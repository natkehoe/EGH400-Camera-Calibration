%% Calculating the area covered by the camera based on init 
% Author: Natalie Kehoe

clc, clear all, close all

load init.mat

% Calculate area covered by camera
thetas = cam.FoV/2;
height = h1;
xy = (height*atan(thetas)) * 2;

area = xy(1)*xy(2);

% Calculate how large image must be
area20percent = area*0.2;
minDimCheckerboard = sqrt(area20percent);

disp(['Camera covers an area of ', num2str(xy(1)) ' x ' num2str(xy(2)) ' metres.'])
disp(['Total area = ', num2str(area) ' metres. Therefore 20% of image is ' num2str(area*0.2) ' metres.'])
disp(['Checkerboard must be at least ' num2str(minDimCheckerboard) ' metres width.'])
