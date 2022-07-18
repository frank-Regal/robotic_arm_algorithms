%% Title:    THA3, Frame to Frame Transformation (PA1 Goal 5)
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2022.04.29
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

function [bTip,bPost]=OptPivotCalibration(dBar,dTilda,Dj,Hj,NF)

Td = FrameToFrameTransformation(dBar,dTilda,Dj,NF);

% Get Size of Hj (Pose of Points w.r.t. EM Tracker)
[Nh,~] = size(Hj);

% Calculate an Arbitrary Frame for Pivot Calibration
hBar = (1/Nh) * sum(Hj(:,1:3));

% Init
hTilda = zeros(Nh,3);

% Calculation of a tilda (ref. slide 11 W11-L1)
for j = 1:Nh
    % Caluclation of Dot Poses w.r.t. First Frame Calculated Above (gBar)
    hTilda(j,:) = Hj(j,1:3) - hBar;
end

% Create a G Readings
Hi= Hj(:,4:(NF*3));

% Create One Less Number of Frames
NF = NF -1;

% Get Fg[k]'s 
Th = FrameToFrameTransformation(hBar,hTilda,Hi,NF);

% Init
T = zeros(4,4*NF);

for i = 1:NF
    % Create Fa and Fd matrices for this frame
    Fh = Th(:,(i*4)-3:(i*4));
    Fd = Td(:,(i*4)-3:(i*4));
    T(:,(i*4)-3:(i*4)) = InverseTransformMatrix(Fd)*Fh;
end

%% Run Pivot Calibration Method (ref. Slide 16 W11-L2)

% Init
p = zeros(3*(NF-1),1);
A = zeros(3*(NF-1),6);

% Parsing Tranformation Matrix 
for f = 1:(NF-1)
    % Grab Rotation Part of Tranformation Matrix Obtained Above
    A((f*3)-2:(f*3),1:3) = T(1:3,(f*4)-3:(f*4)-1);

    % Fill out the rest of A matrix
    A((f*3)-2:(f*3),4:6) = -eye(3);

    % Grab Translation Part of Tranformation Matrix Obtained Above
    p((f*3)-2:(f*3)) = -T(1:3,(f*4));
end

% Least Squares
% b = inv(A'*A)*A'*p;
pos = lsqr(A,p);

% Tip Vector
bTip = pos(1:3);

% Post Vector
bPost = pos(4:6);

end



    