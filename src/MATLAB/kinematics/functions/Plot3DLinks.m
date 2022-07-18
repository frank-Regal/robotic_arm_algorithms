%% Title:    THA 1, Programming Assignment Helper Function
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2021.02.24
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

% Used for plotting unit axes of coordinate transforms
%
% Input:
%     T: 4x4 homogeneous transform
% Output:
%     style: string for either 'origin' or 'base' transform

function Plot3DLinks(T1, T2)

        line([T1(1,4) T2(1,4)],[T1(2,4) T2(2,4)],[T1(3,4) T2(3,4)], 'color', [0.8500 0.3250 0.0980], 'linewidth', 1)
        
end