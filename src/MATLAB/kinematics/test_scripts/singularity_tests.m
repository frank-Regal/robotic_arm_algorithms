%% Title:    THA 2, PA Determining Singularities
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2022.04.05
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

clear;
fprintf("\n\n\n");
disp("////////////////////////////////////////////////////////////////");
disp("================= SINGULARITY TEST SCRIPT ======================");
%% Get List of 3 Singularities (No Constraints on Robot)

% Get Kuka Quantec Parameters With Sybmobilic Joint Angles
[s_hat,q,h,~,theta]=GetKukaQuantecParams("space","true");

% Get a Symbolic Jacobian
j_space = J_space(s_hat,q,h,theta,'makesymbolic');

% Find 10 Singularities
[thetas, IsSingular] = singularity(j_space,theta,'FindMultiple');

% Print Out
fprintf("\nList of Kuka Qunatec Singularities (space jacobian):\n")
for i = 1:length(thetas)
    fprintf("\nSingularity Number %0.0f [rad]:\n T1: %0.4f, T2: %0.4f, " + ...
        "T3: %0.4f, T4: %0.4f, T5: %0.4f, T6: %0.4f",i,thetas(i).t1, ...
        thetas(i).t2, thetas(i).t3, thetas(i).t4, thetas(i).t5, ...
        thetas(i).t6);
end

%% Showing A Singular Config Return of 6DOF Kuka Robot

% Get Kuka Quantec Parameters
[s_hat,q,h,~,~]=GetKukaQuantecParams("space","false");

% Specified Parameters
theta_1= [5*pi/4;-pi/4;pi/2;0;0;pi/4];

% Calculation of the Space Jacobian
j_space = J_space(s_hat,q,h,theta_1);

% Print Out
fprintf("\n\nTesting If Kuka is Singular at Specified Joint Angles:\n")
fprintf("\nJoint Angles (1->6) [rad]:\n")
disp(theta_1');
fprintf("\nSinguarity Response:\n")
[~, ~] = singularity(j_space,theta_1);

%% Showing Not A Singular Config Return of 6DOF Kuka Robot

% Get Kuka Quantec Parameters
[s_hat,q,h,M,~]=GetKukaQuantecParams("space","false");

% Specified Parameters
theta_2= [pi;pi/4;pi;pi/4;pi/2;pi/6];

% Calculation of the Space Jacobian
j_space = J_space(s_hat,q,h,theta_2);

% Print Out
fprintf("\n\nTesting If Kuka is Singular at Specified Joint Angles:\n")
fprintf("\nJoint Angles (1->6) [rad]:\n")
disp(theta_2');
fprintf("\nSinguarity Response:\n")
[~, ~] = singularity(j_space,theta_2);

