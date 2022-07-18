function []=ellipsoid_plot_linear(Jacobian, T,figure_name)

% Turn off Warnings
warning('off','all')

% Grab Eigenvectors and EigenValues
[vectors,lambdas] = GetJacobianEigs(Jacobian,"linear");

% Get Rotation Matrix
R = T(1:3,1:3);

% Assign Variables for Ellipsoid
xc = T(1,4);           % Center of Ellipsoid x-axis
yc = T(2,4);           % Center of Ellipsoid y-axis               
zc = T(3,4);           % Center of Ellipsoid z-axis
xr = sqrt(lambdas(1)); % Ellipsoid SemiAxis Length x-axis
yr = sqrt(lambdas(2)); % Ellipsoid SemiAxis Length y-axis
zr = sqrt(lambdas(3)); % Ellipsoid SemiAxis Length z-axis

% Create a Circle for Base Reference of Ellipsoid
r = 1;                  % radius
n = 1000;               % number of points of circle
t = linspace(0,2*pi,n); % space out points of circle

% // Create Ellipsoid in X-Y Plane
for i = 1:length(t)
    xy_vec(1,i) = xr*r*sin(t(i));
    xy_vec(2,i) = yr*r*cos(t(i));
    xy_vec(3,i) = zr*0;
    xy_vec(:,i) = R*xy_vec(:,i) + [xc;yc;zc];
end
figure(figure_name);
plot3(xy_vec(1,:),xy_vec(2,:),xy_vec(3,:),'g')
line([xc xy_vec(1,1)],[yc xy_vec(2,1)],[zc xy_vec(3,1)],'color', 'g', 'linewidth', 4);
hold on;

% // Create Ellipsoid in X-Z Plane
for i = 1:length(t)
    xz_vec(1,i) = xr*r*cos(t(i));
    xz_vec(2,i) = yr*0;
    xz_vec(3,i) = zr*r*sin(t(i));
    xz_vec(:,i) = R*xz_vec(:,i) + [xc;yc;zc];
end
figure(figure_name);
plot3(xz_vec(1,:),xz_vec(2,:),xz_vec(3,:),'r')
line([xc xz_vec(1,1)],[yc xz_vec(2,1)],[zc xz_vec(3,1)],'color', 'r', 'linewidth', 4);
hold on;

% // Create Ellipsoid in Y-Z Plane
for i = 1:length(t)
    yz_vec(1,i) = xr*0;
    yz_vec(2,i) = yr*r*sin(t(i));
    yz_vec(3,i) = zr*r*cos(t(i));
    yz_vec(:,i) = R*yz_vec(:,i) + [xc;yc;zc];
end
figure(figure_name);
plot3(yz_vec(1,:),yz_vec(2,:),yz_vec(3,:),'b')
line([xc yz_vec(1,1)],[yc yz_vec(2,1)],[zc yz_vec(3,1)],'color', 'b', 'linewidth', 4);
hold on;

% // Plot Variables
figure(figure_name);
end