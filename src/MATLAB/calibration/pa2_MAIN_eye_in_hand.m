%% Title:    THA 2, Programming Assignment Main Script
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2022.04.05
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

close all;
clear;
clc;

%% Part 1: Get Non-Noisy Data
% Grab Non-Noisy Data
[q_Robot_config, q_camera_config,t_Robot_config,t_camera_config] = data_quaternion();

% Get Half the Dataset for Part 2bs
half_q_Robot_config = q_Robot_config(1:5,:);    % Quaternion
half_q_camera_config = q_camera_config(1:5, :); % Quaternion
half_t_Robot_config = t_Robot_config(1:5,:);    % Translation
half_t_camera_config = t_camera_config(1:5, :); % Translation

% Get output Transform X from the camera frame to the robobts end effector
% frame
Tx = GetEyeInHandTF(q_Robot_config, q_camera_config,t_Robot_config,t_camera_config);

% Print Out
fprintf("Transformation Matrix for Non-Noisy Data:\n");
disp(Tx);

%% Part 2a

% Grab Noisy Data
[q_Robot_config_noise, q_camera_config_noise,t_Robot_config_noise,t_camera_config_noise] = data_quaternion_noisy();

% Get Half the Dataset for Part 2bs
half_q_Robot_config_noise = q_Robot_config_noise(1:5,:);    % Quaternion
half_q_camera_config_noise = q_camera_config_noise(1:5, :); % Quaternion
half_t_Robot_config_noise = t_Robot_config_noise(1:5,:);    % Translation
half_t_camera_config_noise = t_camera_config_noise(1:5, :); % Translation 

% Get output Transform X from the camera frame to the robobts end effector
% frame
Tx_noise = GetEyeInHandTF(q_Robot_config_noise, q_camera_config_noise,t_Robot_config_noise,t_camera_config_noise);

% Print Out
fprintf("Transformation Matrix for Noisy Data:\n");
disp(Tx_noise);

%% Part 2b
% Calculating T from half of the noise free data set
Tx_half= GetEyeInHandTF(half_q_Robot_config, half_q_camera_config,half_t_Robot_config,half_t_camera_config);

% Calculating T from half of the noisy data set
Tx_half_noise= GetEyeInHandTF(half_q_Robot_config_noise, half_q_camera_config_noise,half_t_Robot_config_noise,half_t_camera_config_noise);

% Print Out
fprintf("Transformation Matrix for Half of Non-Noisy Data:\n");
disp(Tx_half);

fprintf("Transformation Matrix for Half of Noisy Data:\n");
disp(Tx_half_noise);




