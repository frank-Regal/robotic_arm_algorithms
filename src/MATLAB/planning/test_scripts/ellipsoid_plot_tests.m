%% Title:    THA 2, PA Ellipsoid Plots Test Cases
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2022.04.05
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

clear;
fprintf("\n\n\n");
disp("////////////////////////////////////////////////////////////////");
disp("=============== ELLIPSOID PLOTS TEST SCRIPT ====================");
%% Showing Ellipsoid Plots at Kuka Home Configuration

% Start New Figure
fig1 = figure();

% Get Kuka Parameters
[s_hat,q,h,M,theta]=GetKukaQuantecParams("body","false");

% Get Forward Kinematics of body
Tsb = FK_body(s_hat,q,h,M,theta,'PlotMe');

% Get Body Jacobian
body_j = J_body(s_hat,q,h,theta);

% Plotting Ellipsoid for Linear and Angular
ellipsoid_plot_linear(body_j,Tsb,fig1);
ellipsoid_plot_angular(body_j,Tsb,fig1);

% Caluclating Isotropy, Condition, and Ellipsoid Volume
[~,vals_ang] = GetJacobianEigs(body_j,"angular");
[~,vals_lin] = GetJacobianEigs(body_j,"linear");

% Find Isotropy
isotropy_ang = J_isotropy(vals_ang);
isotropy_lin = J_isotropy(vals_lin);

% Find Condition Number
cond_num_ang = J_condition(vals_ang);
cond_num_lin = J_condition(vals_lin);

% Find Volume of Ellipsoid
vol_ang = J_ellipsoid_volume(vals_ang);
vol_lin = J_ellipsoid_volume(vals_lin);

% Print out
fprintf("\nKuka Manipulability Parameters (At Home Configuration):\n");
fprintf("\nIsotropy for Angular: %0.4f; for Linear: %0.4f\n",isotropy_ang,isotropy_lin);
fprintf("\nCondition Number for Angular: %0.4f; for Linear: %0.4f\n",cond_num_ang,cond_num_lin);
fprintf("\nVolume of Ellipsoid for Angular: %0.4f; for Linear: %0.4f\n",vol_ang,vol_lin);

hold off;

%% Showing Ellipsoid Plots at Kuka Singularity
% Start New Figure
fig2 = figure();

% Get Kuka Parameters
[s_hat,q,h,M,~]=GetKukaQuantecParams("body","false");

% Singularity Joint Angles
theta_1= [5*pi/4;-pi/4;pi/2;0;0;pi/4];

% Get Forward Kinematics of body
Tsb = FK_body(s_hat,q,h,M,theta_1,'PlotMe');

% Get Body Jacobian
body_j = J_body(s_hat,q,h,theta_1);

% Plotting Ellipsoid for Linear and Angular
ellipsoid_plot_linear(body_j,Tsb,fig2);
ellipsoid_plot_angular(body_j,Tsb,fig2);

% Caluclating Isotropy, Condition, and Ellipsoid Volume
[~,vals_ang] = GetJacobianEigs(body_j,"angular");
[~,vals_lin] = GetJacobianEigs(body_j,"linear");

% Find Isotropy
isotropy_ang = J_isotropy(vals_ang);
isotropy_lin = J_isotropy(vals_lin);

% Find Condition Number
cond_num_ang = J_condition(vals_ang);
cond_num_lin = J_condition(vals_lin);

% Find Volume of Ellipsoid
vol_ang = J_ellipsoid_volume(vals_ang);
vol_lin = J_ellipsoid_volume(vals_lin);

% Print out
fprintf("\n\nKuka Manipulability Parameters (At Singularity Configuration):\n");
fprintf("\nIsotropy for Angular: %0.4f; for Linear: %0.4f\n",isotropy_ang,isotropy_lin);
fprintf("\nCondition Number for Angular: %0.4f; for Linear: %0.4f\n",cond_num_ang,cond_num_lin);
fprintf("\nVolume of Ellipsoid for Angular: %0.4f; for Linear: %0.4f\n",vol_ang,vol_lin);

hold off;