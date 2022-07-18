%% Title:    THA 2, Right and Left Jacobian Transposes
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2022.04.05
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

% Creates either the Right or Left Transpose
% Ref: Week 8 Lecture 2 slides 4 through 5

% Inputs:
%    J: Current frames Jacobian
%    side: Right or left 

% Outputs:
%    J_pseudo: psuedoinverse of the Jacobian for the given side
function J_pseudo=J_dagger(J,side)

% Determine which side is required
if (side == "right")
    % Calculate right Transpose
    J_pseudo = transpose(J) * inv(J*transpose(J));
elseif (side == "left")
    % Calculate Left Transpose
    J_pseudo = inv(transpose(J)*J)*transpose(J);
end

end