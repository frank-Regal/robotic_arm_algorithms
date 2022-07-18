%% Title:    THA3, Frame to Frame Transformation (PA1 Goal 1)
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2022.04.29
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

function [T] = PSToPSRegistration(PointSet, NumFrames)

% Size of Data Set
[N, ~] = size(PointSet);

% Init
H = zeros(3,3);
T = zeros(4,4*(NumFrames-1));
aTilda = zeros(N,3);
bTilda = zeros(N,3);

% Compute Centroid of Objects per frame
for i = 1:(NumFrames-1)
    aBar = (1/N) * sum(PointSet(1:N,(i*3)-2:(i*3)));
    bBar = (1/N) * sum(PointSet(1:N,((i+1)*3)-2:((i+1)*3)));

    for j = 1:N
        % Create a and b tilda
        aTilda(j,:) = PointSet(j,(i*3)-2:(i*3)) - aBar;
        bTilda(j,:) = PointSet(j,((i+1)*3)-2:((i+1)*3)) - bBar;

        % Build H Matrix
        m = [aTilda(j,1)*bTilda(j,1) aTilda(j,1)*bTilda(j,2) aTilda(j,1)*bTilda(j,3);
            aTilda(j,2)*bTilda(j,1) aTilda(j,2)*bTilda(j,2) aTilda(j,2)*bTilda(j,3);
            aTilda(j,3)*bTilda(j,1) aTilda(j,3)*bTilda(j,2) aTilda(j,3)*bTilda(j,3)];
        H = H + m;
    end

    % Get SVD of H Matrix
    [U,~,V] = svd(H);

    % Calculate R of Matrix
    R = V*U';

    % Make sure determinate is R
    if (round(det(R)) ~= 1)
        disp("WARNING: Pontential Failure");
    end

    % Calculate Position
    p = bBar'-R*aBar';

    % Calculate Transform In Between Each Frame
    T(:,(i*4)-3:(i*4)) = [R p;
        0 0 0 1];
end
end