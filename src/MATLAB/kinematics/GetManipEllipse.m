function [] = GetManipEllipse(Tsb, body_j,fig1)

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


fprintf("\nKuka Manipulability Parameters:\n");
fprintf("\nIsotropy for Angular: %0.4f; for Linear: %0.4f\n",isotropy_ang,isotropy_lin);
fprintf("\nCondition Number for Angular: %0.4f; for Linear: %0.4f\n",cond_num_ang,cond_num_lin);
fprintf("\nVolume of Ellipsoid for Angular: %0.4f; for Linear: %0.4f\n",vol_ang,vol_lin);
hold on
end