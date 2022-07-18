%% Title:    THA 2, Ellipsoid Condition Number
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2022.04.05
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

% Determines the condition number of the given manipulability ellipsoid 
% Ref: Week 7 Lecture 2 slide 17

% Inputs:
%    lamdas: The Eigenvalues of the manipulability ellipsoid 

% Outputs:
%    condition_number: the condition number of the manipulability ellipsoid 

function condition_number=J_condition(lambdas)

% Find the largest and smallest eigenvalues in the set
max_val = max(lambdas);
min_val = min(lambdas);

% Calculate the condition number
condition_number = max_val/min_val;

end