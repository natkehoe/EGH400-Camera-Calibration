%% CAMERA CALIBRATION
clc, clear all, close all

addpath("./Functions/")


%% 1 Get calibration intrinsics and extrinsics
revision = '05';
calibrationDataMat = "calibrationSession16042025.mat"; % filename of camera calibration
calibSession = load(calibrationDataMat);

check2cam = calibSession.calibrationSession.CameraParameters.PatternExtrinsics(end,1)
imageName = calibSession.calibrationSession.PatternSet.PatternLabels(end)

invA = inv(check2cam.A) % pattern to camera

format long
checkerboard2camera_t = invA(1:3, 4)
checkerboard2camera_R = invA(1:3, 1:3);
checkerboard2camera_q = quaternion(checkerboard2camera_R, 'rotmat', 'frame')


% camera intrinsics
camIntrinsicsMatrix =  calibSession.calibrationSession.CameraParameters.Intrinsics.K


% distCoeffs = [k1, k2, p1, p2]
radialDistortion = calibSession.calibrationSession.CameraParameters.RadialDistortion;
tangentialDistortion = calibSession.calibrationSession.CameraParameters.TangentialDistortion;

distCoeffs = [radialDistortion, tangentialDistortion]