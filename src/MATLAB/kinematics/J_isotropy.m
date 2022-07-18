%% Title:    THA 2, Isotorpy of Manipulability Ellipsoid
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2022.04.05
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

% Determines the isotropy of the given manipulability ellipsoid 
% Ref: Week 7 Lecture 2 slide 17

% Inputs:
%    lamdas: The Eigenvalues of the manipulability ellipsoid 

% Outputs:
%    isotorpy: The isotropy of the manipulability ellipsoid 
function isotropy=J_isotropy(lambdas)

% Find largest and smallest eigenvalue in the set
max_val = max(lambdas);
min_val = min(lambdas);

% Detemine isotorpy
isotropy = sqrt(max_val)/sqrt(min_val);
    
end