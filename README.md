# EGH400-Camera-Calibration
Thesis Project - Camera Calibration Testing for Localisation System

This repository stores data gathered for calibrating the Intel Realsense LiDAR Camera L515, in preparation for testing the ROS2-Localisation-System. 
A proper calibration procedure is required to accurately determine the position of the ARUCO marker, and minimise the potential noise coming from the camera's readings,
which can impact the tracking of the aruco marker by giving false readings of the marker position.

MATLAB scripts are created for calibration and comparison with OptiTracker results. Outputs of these scripts include...
- Camera matrix from the calibration session
- World Transformations

## `extrinsics_test.m`
Used to analyse the checkerboard marker image; outputs the checkerboard origin position.

## TEST1_CameraCalibration1

### `WorldTransfromations11.m`
Creates the 3D environment of the Optitracker global position; camera position (estimated); 
optitracker output and the checkerboard marker position (based on camera intrinsics).
By comparing the optitracker output and camera output, the precise camera position can be determined.
