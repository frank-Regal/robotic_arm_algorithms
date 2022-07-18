%% Title:    THA 1, Programming Assignment Problem 1.A.
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2021.02.24
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

% Convert a Rotation Matrix to an Axis and Angle Representation
%
% Input:
%   R: 3x3 SO(3) rotation matrix
%
% Output:
%   omega: 3x1 unit vector defining the axis of rotation
%   theta: (rad) angle of rotation about the axis of rotation
%
% Reference:
% Lynch, Modern Robotics, Chapter 3, Page 87, Algorithm
% for equations 3.58 through 3.61

function [theta,omega]=RotToAxisAngle(R)
    % Calculate the trace of the rotation matrix
    t = R(1,1) + R(2,2) + R(3,3);

    % if no rotation
    if t == 3
        disp("ERROR: No Rotation theta: 0; omega: Undefined");
        theta = 0;
        omega = nan;

    % not a unique solution, this is surjective onto the space of rotations
    elseif t == -1
        theta = pi();
        
        option1 = (1/sqrt(2*(1 + R(3,3))));
        option2 = (1/sqrt(2*(1 + R(2,2))));
        option3 = (1/sqrt(2*(1 + R(1,1))));
    
        if ~isinf(option1)
            omega =  option1 * [  R(1,3);   R(2,3); (1+R(3,3))];
        elseif ~isinf(option2)
            omega =  option2 * [  R(1,2); 1+R(2,2);     R(3,2)];
        elseif ~isnf(option3)
            omega =  option3 * [1+R(1,1);   R(2,1);     R(3,1)];
        else
            disp("ERROR: RotToAxisAngle Function");
        end 

    % otherwise
    else 
        theta = acos(0.5*(t - 1));
        omega_skew = (1/(2*sin(theta))) * (R-R');
        
        % Convert to a vector
        omega = SkewMatrixToVector(omega_skew);
    end
    
    
end