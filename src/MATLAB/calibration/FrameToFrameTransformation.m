%% Title:    THA3, Frame to Frame Transformation (PA1)
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2022.04.29
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

function [T]=FrameToFrameTransformation(aBar,aTilda,bi,NF)

% Size of Data Set
[Nb, ~] = size(bi);

% Init
H = zeros(3,3);
T = zeros(4,4*NF);
bTilda = zeros(Nb,3);

% Compute Centroid of Objects per frame
for i = 1:NF
    % Calculate the Centroid of Read Data
    bBar = (1/Nb) * sum(bi(1:Nb,(i*3)-2:(i*3)));

    for j = 1:Nb
        % Create the distance from Centroid on Read Data
        bTilda(j,:) = bi(j,((i*3)-2:(i*3))) - bBar;

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

    % Make sure determinate is 1 or Negative 1
    if (round(det(R)) == -1)
        V(:,3) = -V(:,3);
        R = V*U';
    elseif (round(det(R)) ~= 1)
        disp("WARNING: Pontential Failure");
    end

    % Calculate Position
    p = bBar'-R*aBar';

    % Calculate Transform In Between Each Frame
    T(:,(i*4)-3:(i*4)) = [R p;
                          0 0 0 1];
end

end