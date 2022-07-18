%% Title:    THA 2, Programming Assignment Main Script
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2022.04.05
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

close all;
clear;
clc;

% ***********************************************************
% // Comment/Uncomment Scripts you do/do not want to run
% // All test script files are located in test_scripts folder
% ***********************************************************

%% Part B - Space Frame Forward Kinematics

% //Run Test Script for Space Jacobian
run("space_fk_tests.m");
pause(2);

%% Part C - Body Frame Forward Kinematics

% //Run Test Script for Body Forward Kinematics
run("body_fk_tests.m");
pause(2);

%% Part E - Calculating the Jacobians

% // Run Test Script for Space Jacobian
run("space_jacobian_tests.m");

% // Run Test Script for Body Jacobian
run("body_jacobian_tests.m");
pause(15);

%% Part F - Singularity Analysis

% // Run Test Script for Singularity Tests
run("singularity_tests.m");
pause(35);

%% Part G - Visualizing the Jacobian (Manipuability Ellipsoids)

% // Run Test Script for Visualizing the Jacobian with Manip Ellipsoids
run("ellipsoid_plot_tests.m")
pause(5);

%% Part H - Inverse Kinematics

% // Run Test Script for Calculating Inverse Kinematics
run("inverse_kinematics_tests.m");
pause(10);

%% Part I - Jacobian Transpose Kinematics

% // Run Test Script for Calculating Jacobian Transpose Kinematics
run("jacobian_transpose_tests.m")
pause(15);

%% Part K - DLS Inverse Kinematics

% // Run Test Script for Calculating DLS Inverse Kinematics
run("dls_inverse_tests.m")