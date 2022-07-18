%% Title:    THA 1, Programming Assignment Problem 2.B.
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2021.02.24
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

% Convert a Quaternion to Rotation Matrix
% Ref: Lynch, Modern Robotics

% Inputs:
%    Q: 4x1 vector representing q0, q1, q2, q3 of a quaternion 

% Outputs:
%    RotMat: 3x3 rotation matrix

function RotMat=QuaternionToRot(Q)

% Check for singularity
[theta,w] = QuaternionToAxisAngle(Q);
if (theta == pi)
    RotMat = [nan nan nan;
        nan nan nan;
        nan nan nan];
else
    % Check for out of (-pi, pi) bounds Quaternion
    if (theta > pi || theta < -pi)
        theta = GetInRange(theta);
        Q = AxisAngleToQuaternion(w,theta);
    end

    % Convert to Unit Quaternion for Formulation
    mag = sqrt((Q(1)^2) + (Q(2)^2) + (Q(3)^2) + (Q(4)^2));
    Q = Q/mag;

    % Equation B.12, Lynch, Modern Robotics
    % Column 1
    elm11 = (Q(1)^2) + (Q(2)^2) - (Q(3)^2) - (Q(4)^2);
    elm21 = 2*((Q(1)*Q(4)) + (Q(2)*Q(3)));
    elm31 = 2*((Q(2)*Q(4)) - (Q(1)*Q(3)));

    % Column 2
    elm12 = 2*((Q(2)*Q(3)) - (Q(1)*Q(4)));
    elm22 = (Q(1)^2) - (Q(2)^2) + (Q(3)^2) - (Q(4)^2);
    elm32 = 2*((Q(1)*Q(2)) + (Q(3)*Q(4)));

    % Column 3
    elm13 = 2*((Q(1)*Q(3)) + (Q(2)*Q(4)));
    elm23 = 2*((Q(3)*Q(4)) - (Q(1)*Q(2)));
    elm33 = (Q(1)^2) - (Q(2)^2) - (Q(3)^2) + (Q(4)^2);

    % Combining
    RotMat = [elm11 elm12 elm13;
        elm21 elm22 elm23;
        elm31 elm32 elm33];
end
end

% Function to put theta in the allowable range of -pi to pi
function output=GetInRange(theta)
    if (theta > pi)
        while theta > pi
            theta = theta - 2*pi;
        end
    elseif (theta < -pi)
        while theta < -pi
            theta = theta + 2*pi;
        end 
    end 

    output = theta;
end