%% Title:    THA 1, Programming Assignment - Screw Axis to Components
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2021.02.24
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

% Convert a screw axis (S) to its components {q,s,h}

% Input:
%     S: 6x1 screw axis with angular and linear velocity
%
% Output:
%     q: 3x1 vector that goes from the ref frame to a point perpendicular 
%        on the screw axis
%     s: 3x1 unit vector defining the screw axis
%     h: pitch of the the screw

function [q,s,h]=ScrewAxisToComponents(S)

% Lecture Slide W5-L1, Slide 5
w = S(1:3);
v = S(4:6);

% Get skew symmetric matrix for solving cross product
w_skew = VectorToSkewMatrix(w);

% Calculate magnitude of angular and linear velocity
mag_w = sqrt(w(1)^2 + w(2)^2 + w(3)^2);
mag_v = sqrt(v(1)^2 + v(2)^2 + v(3)^2);

if (mag_w == 1)
    % Pure Rotation Case
    h = 0;
    q = -w_skew'*v; 
    s = w;
elseif (mag_v == 1 && mag_w == 0)
    h = infinity;
    s = w;
    q = 0;
else 
    h = mag_v/mag_w;
    q = -w_skew'*(v-(h*w));
    s = w;
end 

end