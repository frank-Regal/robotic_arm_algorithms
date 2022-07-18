%% Title:    THA 2, PA Space Frame Forward Kinematics
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2022.04.05
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

clear;
fprintf("\n\n\n");
disp("////////////////////////////////////////////////////////////////");
disp("=================== FK SPACE TEST SCRIPT =======================");
%% Get Tsb of Kuka Quantec

% Get Kuka Quantec Parameters
[s_hat,q,h,M,theta_kuka]=GetKukaQuantecParams("space","false");

% Space Frame Forward Kinematics
figure();
Tsb_kuka = FK_space(s_hat,q,h,M,theta_kuka,'PlotMe');
hold off;

% Print out
fprintf("\nKuka Qunatec Forward Kinematics (space frame):\n\n")
fprintf("\nTF (Body Frame w.r.t Space Frame):\n\n")
disp(Tsb_kuka);
fprintf('\nJoint Angles (1->6) [rad]:\n\n')
disp(theta_kuka);

%% Get Tsb of Barrett Technology's WAM 7R Robot

% Parameters Given in Modern Robotics (Lynch)
% Get Barrett Technology's WAM 7R Robot Params
[s_hat,q,h,M,theta_wam]=GetBarrettTechWAMParams("space","false");

% Space Frame Forward Kinematics
Tsb_wam = FK_space(s_hat,q,h,M,theta_wam);

% Print out
fprintf("\nBrarett WAM Forward Kinematics (space frame):\n\n")
fprintf("\nTF (Body Frame w.r.t Space Frame):\n\n")
disp(Tsb_wam);
fprintf('\nJoint Angles (1->7) [rad]:\n\n')
disp(theta_wam);

%% Get Tsb of UR5 Robot

% Parameters Given in Modern Robotics (Lynch)
% Get UR5 Robot Params
[s_hat,q,h,M,theta_ur5]=GetUR5Params("space","false");

% Modern Robotics (Kevin Lynch) Example 4.5 Angles Given
example_thetas = [0, -pi/2, 0, 0, pi/2, 0];

% Space Frame Forward Kinematics
Tsb_ur5 = FK_space(s_hat,q,h,M,example_thetas);

% Print out
fprintf("\nUR5 Forward Kinematics (space frame):\n\n")
fprintf("\nNOTE: Example 4.5 in Modern Robotics (Lynch) \n");
fprintf("\nTF (Body Frame w.r.t Space Frame):\n\n")
disp(Tsb_ur5);
fprintf('\nJoint Angles (1->6) [rad]:\n\n')
disp(example_thetas);
