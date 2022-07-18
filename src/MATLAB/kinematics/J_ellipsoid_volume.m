%% Title:    THA 2, Ellipsoid Volume
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2022.04.05
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

% Determines the volume of the given manipulability ellipsoid 
% Ref: Week 7 Lecture 2 slide 17

% Inputs:
%    lamdas: The Eigenvalues of the manipulability ellipsoid 

% Outputs:
%    Volume: the Volume of the manipulability ellipsoid 

function elip_vol=J_ellipsoid_volume(lambdas)

% Set starting value to one as base
prod = 1;

% Loop through lambdas and multiply each to current product value
for i = 1:length(lambdas)
    prod = prod * lambdas(i); 
end

% Take square root of final product value
elip_vol = sqrt(prod);
end