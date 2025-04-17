%% CAMERA CALIBRATION
clc, clear all, close all

addpath("./Functions/")

%% 1 Get calibration intrinsics and extrinsics
revision = '05';
calibrationDataMat = "calibrationSession16042025.mat"; % filename of camera calibration
calibSession = load(calibrationDataMat);

check2cam = calibSession.calibrationSession.CameraParameters.PatternExtrinsics(end,1)

invA = inv(check2cam.A) % pattern to camera

format long
checkerboard2camera_t = invA(1:3, 4)
checkerboard2camera_R = invA(1:3, 1:3);
checkerboard2camera_q = quaternion(checkerboard2camera_R, 'rotmat', 'frame')


% camera intrinsics
camIntrinsicsMatrix =  calibSession.calibrationSession.CameraParameters.Intrinsics.K