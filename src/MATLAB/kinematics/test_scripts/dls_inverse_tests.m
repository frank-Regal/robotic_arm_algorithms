%% Title:    THA 2, PA DLS Inverse Test Script
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2022.04.05
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

clear;
fprintf("\n\n\n");
disp("////////////////////////////////////////////////////////////////");
disp("=================== DLS INVERSE TEST SCRIPT ====================");
% %% [Test 1] Trying to Reach Signularity Location with Kuka Quantec
% 
% % Get Kuka Quantec Parameters
% [s_hat,q,h,M,~]=GetKukaQuantecParams("body","false");
% 
% % Find Transform from Space frame of Arbitrary Location 
% theta = [5*pi/4;-pi/4;pi/2;0;0;pi/4]; % Joint Angles Just used to Find TF
% Tsd = FK_body(s_hat,q,h,M,theta);     % TF of Desired End Effect Location
% 
% % Initial Joint Angle Guess
% init_theta = [0;0;0;0;pi/4;0];
% 
% % Run J Inverse Kinematics
% fig1 = figure();
% joint_angles = DLS_inverse_kinematics(Tsd,init_theta,fig1);
% title("Using DLS for Inv Kinematics of Kuka (Part K): Singularity");
% hold off;
% 
% % Print Out
% fprintf("\n[Test 1] DLS Inverse Kinematics found the following Joint Angles will Reach Desired Configuration:\n\n")
% fprintf('\nResulting Joint Angles (1->6) [rad]:\n\n')
% disp(joint_angles');
% fprintf("\nDesired Configuration of End Effector:\n\n")
% disp(Tsd);

%% [Test 2] Trying to Reach Random Location with Kuka Quantec

% Get Kuka Quantec Parameters
[s_hat,q,h,M,~]=GetKukaQuantecParams("body","false");

% Find Transform from Space frame of Arbitrary Location 
theta = [3*pi/4;-pi/4;0;-pi/4;0;0]; % Joint Angles Just used to Find TF
Tsd = FK_body(s_hat,q,h,M,theta);     % TF of Desired End Effect Location

% Initial Joint Angle Guess
init_theta = [0;0;0;0;pi/4;0];

% Run J Inverse Kinematics
fig2 = figure();
joint_angles = DLS_inverse_kinematics(Tsd,init_theta,fig2);
title("Using DLS for Inv Kinematics of Kuka (Part K): Random Location");
hold off;

% Print Out
fprintf("\n[Test 2] DLS Inverse Kinematics found the following Joint Angles will Reach Desired Configuration:\n\n")
fprintf('\nResulting Joint Angles (1->6) [rad]:\n\n')
disp(joint_angles');
fprintf("\nDesired Configuration of End Effector:\n\n")
disp(Tsd);