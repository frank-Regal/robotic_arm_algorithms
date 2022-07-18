%% Title:    THA 2, PA Body Frame Forward Kinematics
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2022.04.05
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

clear;
fprintf("\n\n\n");
disp("////////////////////////////////////////////////////////////////");
disp("=================== FK BODY TEST SCRIPT =======================");
%% Get Tsb of Kuka Quantec

% Get Kuka Quantec Parameters
[s_hat,q,h,M,theta_kuka]=GetKukaQuantecParams("body","false");

% Body Frame Forward Kinematics
kuka_body_fk = figure();
Tsb_kuka = FK_body(s_hat,q,h,M,theta_kuka, 'PlotMe');
title("Kuka Quantec Home Configuration w/ Screw Axes")
hold off;

% Print out
fprintf("\nKuka Qunatec Forward Kinematics (body frame):\n\n")
fprintf("\nTF (Body Frame w.r.t Space Frame):\n\n")
disp(Tsb_kuka);
fprintf('\nJoint Angles (1->6) [rad]:\n\n')
disp(theta_kuka);

%% Get Tsb of Barrett Technology's WAM 7R Robot

% Parameters Given in Modern Robotics (Lynch)
% Get Barrett Technology's WAM 7R Robot Params
[s_hat,q,h,M,theta_wam]=GetBarrettTechWAMParams("body","false");

% Body Frame Forward Kinematics
Tsb_wam = FK_body(s_hat,q,h,M,theta_wam);

% Print out
fprintf("\nBrarett WAM Forward Kinematics (body frame):\n\n")
fprintf("\nTF (Body Frame w.r.t Space Frame):\n\n")
disp(Tsb_wam);
fprintf('\nJoint Angles (1->7) [rad]:\n\n')
disp(theta_wam);

%% Get Tsb of UR5 Robot

% Parameters Given in Modern Robotics (Lynch)
% Get UR5 Robot Params
[s_hat,q,h,M,theta_ur5]=GetUR5Params("body","false");

% Modern Robotics (Kevin Lynch) Example 4.5 Angles Given
example_thetas = [0, -pi/2, 0, 0, pi/2, 0];

% Body Frame Forward Kinematics
Tsb_ur5 = FK_body(s_hat,q,h,M,example_thetas);

% Print out
fprintf("\nUR5 Forward Kinematics (body frame):\n\n")
fprintf("\nNOTE: Example 4.5 in Modern Robotics (Lynch) \n");
fprintf("\nTF (Body Frame w.r.t Space Frame):\n\n")
disp(Tsb_ur5);
fprintf('\nJoint Angles (1->6) [rad]:\n\n')
disp(example_thetas);
