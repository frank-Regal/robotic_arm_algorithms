%% Title:    THA3, Function for Ci Expected (PA1 Goal 3)
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2022.04.29
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

function [CiExpect]=CalcCiExpected(Ta,Td,ci,NF)
% Get Number of Rows
[Nc,~] = size(ci);

% Init 
CiExpect = zeros(Nc,3*NF);

% Build CiExpected Matrix

for i = 1:NF

    % Create Fa and Fd matrices for this frame
    Fa = Ta(:,(i*4)-3:(i*4));
    Fd = Td(:,(i*4)-3:(i*4));

    for j = 1:Nc
        cTemp = [ci(j,:)';1];
        C = InverseTransformMatrix(Fd)*Fa*cTemp;
        CiExpect(j,(i*3)-2:(i*3)) = C(1:3)';
    end

end