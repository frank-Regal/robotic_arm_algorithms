%% Title:    THA 1, Programming Assignment - Calculating Screw Exponential
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2021.02.24
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

% This function finds the exp([S]*Theta) from the definition of a screw 
% axis components

% Input:
%    q: 3x1 vector that goes from the ref frame to a point perpendicular 
%       on the screw axis
%    s_hat: 3x1 unit vector defining the screw axis
%    h: pitch of the the screw
%    theta: scalar for the distance traveled on screw axis (rad)
%
% Output:
%    expST: 4x4 matrix describing the equvalient transformation
%    S: 6x1 screw vector composed of angular velocity for S(1:3) and 
%       linear velocity components for S(4:6)
%    theta: [rad] scalar theta that is found from input

function [expST,S,theta]=CalculateScrewExp(q,s_hat,h,theta)

% Defining Screw Axis Variables Needed for Calculations

% angular velocity
w = s_hat; 
% skew symmetric matric for screw vector
s_skew = VectorToSkewMatrix(s_hat); 
% linear velocity
v = -s_skew*q + h*s_hat;

% Output 6x1 screw vector
S = [w;v]; 

% skew symmetric matrix for w. Redefined for variable consitency
w_skew = VectorToSkewMatrix(w);
% square of skew symmetrtic matrix
w_skew_sqr = w_skew*w_skew;

% Solving for exp([S]Theta)
% Create an identity matrix for calcs
I = eye(3,3);

% Equation 3.51 Modern Robotics, Lynch, Rodrigues Formula, Page 84 
elm11 = I + sin(theta)*w_skew + (1-cos(theta))*(w_skew_sqr);

% Equation 3.88 Modern Robotics, Lynch
elm12 = (I*theta + (1-cos(theta))*w_skew + (theta - sin(theta))*w_skew_sqr)*v;
elm21 = [0 0 0];
elm22 = 1;

% Ouput for Matrix Form
expST= [elm11 elm12;
        elm21 elm22];

end