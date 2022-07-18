%% [Test 1] Part 1 Linear Optimization with only translation and joint limit constraints

% Get Kuka Quantec Parameters
[s_hat,q,h,M,~]=GetKukaQuantecParams("space","false");

% Find Transform from space frame of Arbitrary Location 
theta = [pi/2;0;0;0;pi/4;0]; % Joint Angles Just used to Find TF
Tsd = FK_space(s_hat,q,h,M,theta);     % TF of Desired End Effect Location

% Initial Joint Angle Guess
init_theta = [0;0;0;0;pi/4;0];

% Run J Inverse Kinematics
fig1 = figure();
joint_angles = Linear_Solver_kinematics(Tsd,init_theta,fig1);
title("Translation Constraints");
hold on

%% [Test 2] Part 2 Linear Optimization with translation, rotation, and joint limit constraints

% Get Kuka Quantec Parameters
[s_hat,q,h,M,~]=GetKukaQuantecParams("space","false");

% Find Transform from space frame of Arbitrary Location 
theta = [0;0;pi/2;0;pi/4;0]; % Joint Angles Just used to Find TF
Tsd = FK_space(s_hat,q,h,M,theta);     % TF of Desired End Effect Location

% Initial Joint Angle Guess
init_theta = [0;0;0;0;pi/4;0];

% Run J Inverse Kinematics
fig2 = figure();
joint_angles = Linear_Solver_With_Rotation_kinematics(Tsd,init_theta,fig2);
title("Translation and Rotation Constraints");
hold on

%% [Test 3] Part 1 Linear Optimization with Virtual Wall beyond target

% Get Kuka Quantec Parameters
[s_hat,q,h,M,~]=GetKukaQuantecParams("space","false");

% Find Transform from space frame of Arbitrary Location 
theta = [0;0;pi/2;0;pi/4;0]; % Joint Angles Just used to Find TF
Tsd = FK_space(s_hat,q,h,M,theta);     % TF of Desired End Effect Location

% Initial Joint Angle Guess
init_theta = [0;0;0;0;pi/4;0];

% Run J Inverse Kinematics
fig3 = figure();
joint_angles = Linear_Solver_With_Rotation_VF_kinematics(Tsd,init_theta,fig3,15.7);
title("Virtual Wall Beyond Target Point");
hold on

%% [Test 4] Part 1 Linear Optimization with Virtual Wall in the way of target

% Get Kuka Quantec Parameters
[s_hat,q,h,M,~]=GetKukaQuantecParams("space","false");

% Find Transform from space frame of Arbitrary Location 
theta = [0;0;pi/2;0;pi/4;0]; % Joint Angles Just used to Find TF
Tsd = FK_space(s_hat,q,h,M,theta);     % TF of Desired End Effect Location

% Initial Joint Angle Guess
init_theta = [0;0;0;0;pi/4;0];

% Run J Inverse Kinematics
fig4 = figure();
joint_angles = Linear_Solver_With_Rotation_VF_kinematics(Tsd,init_theta,fig4,251);
title("Virtual Wall Inbetween Robot and Target Point");
hold on