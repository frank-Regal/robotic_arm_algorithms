%% Title:    THA 2, PA Body Frame Jacobian Tests
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2022.04.05
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

clear;
fprintf("\n\n\n");
disp("////////////////////////////////////////////////////////////////");
disp("================= BODY JACOBIAN TEST SCRIPT ====================");
%% Get Both Symbolic & Evaluated Body Jacobian of Kuka Quantec

% Get Kuka Quantec Parameters
[s_hat,q,h,~,theta_kuka_zero]=GetKukaQuantecParams("body","false");

% Real and Symbolic Space Jacobian
syms t1 t2 t3 t4 t5 t6
theta_sym = [t1, t2, t3, t4, t5, t6];
J_kuka_sym = J_body(s_hat,q,h,theta_sym,'makesymbolic');
J_kuka = J_body(s_hat,q,h,theta_kuka_zero);

% Print out
fprintf("\nKuka Qunatec Symbolic Body Jacobian:\n")
disp(simplify(J_kuka_sym));
fprintf("\nKuka Qunatec Body Jacobian at Zero Configuration:\n")
disp(J_kuka);


%% Get Both Symbolic & Evaluated Body Jacobian of Barrett Technology's WAM 7R Robot

% Parameters Given in Modern Robotics (Lynch)
% Get Barrett Technology's WAM 7R Robot Params
[s_hat,q,h,~,theta_wam_zero]=GetBarrettTechWAMParams("body","false");

% Joint Angles from Modern Robotics (Lynch) Example 4.7
theta_wam = [0,pi/4,0,-pi/4,0,-pi/2,0];

% Real and Symbolic Space Jacobian
syms t7
theta_sym_wam = [t1, t2, t3, t4, t5, t6, t7];
J_wam_sym = J_body(s_hat,q,h,theta_sym_wam,'makesymbolic');
J_wam = J_body(s_hat,q,h,theta_wam);
J_wam_zero = J_body(s_hat,q,h,theta_wam_zero);

% Print out
fprintf("\nBarrett WAM 7R Symbolic Body Jacobian:\n")
disp(J_wam_sym);

fprintf("\nBarrett WAM 7R Body Jacobian at Zero Configuration:\n")
disp(J_wam_zero);

fprintf("\nBarrett WAM 7R Body Jacobian at Specified Joint Angles:\n")
fprintf('\nJoint Angles (1->7) [rad]:\n\n');
disp(theta_wam);
fprintf('\nBody Jacobian:\n\n');
disp(J_wam);



%% Get Both Symbolic & Evaluated Body Jacobian of UR5 Robot

% Get UR5 Robot Params
[s_hat,q,h,M,theta_ur5_zero]=GetUR5Params("space","false");

% Modern Robotics (Kevin Lynch) Example 4.5 Angles Given
theta_ur5 = [0,-pi/2,0,0,pi/2,0];

% Real and Symbolic Space Jacobian
J_ur5_sym = J_body(s_hat,q,h,theta_sym,'makesymbolic');
J_ur5_zero = J_body(s_hat,q,h,theta_ur5_zero);
J_ur5 = J_body(s_hat,q,h,theta_ur5);

% Print out
fprintf("\nUR5 Symbolic Body Jacobian:\n")
disp(simplify(J_ur5_sym));

fprintf("\nUR5 Body Jacobian at Zero Configuration:\n")
disp(J_ur5_zero);

fprintf("\nUR5 Body Jacobian at Speficied Joint Angles:\n")
fprintf('\nJoint Angles (1->6) [rad]:\n\n')
disp(theta_ur5);
fprintf('\nBody Jacobian:\n\n');
disp(J_ur5);



