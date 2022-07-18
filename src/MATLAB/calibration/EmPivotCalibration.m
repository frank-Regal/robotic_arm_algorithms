%% Title:    THA3, Function for EM Pivot Calibration (PA1 Goal 4)
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2022.04.29
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

function [bTip,bPost]=EmPivotCalibration(GJ,NFrames)

% Get Size of GJ (Pose of Points w.r.t. EM Tracker)
[Ng,~] = size(GJ);

% Calculate an Arbitrary Frame for Pivot Calibration
gBar = (1/Ng) * sum(GJ(:,1:3));

% Init
gTilda = zeros(Ng,3);

% Calculation of a tilda (ref. slide 11 W11-L1)
for j = 1:Ng
    % Caluclation of Dot Poses w.r.t. First Frame Calculated Above (gBar)
    gTilda(j,:) = GJ(j,1:3) - gBar;
end

% Create a G Readings
Gi= GJ(:,4:(NFrames*3));

% Create One Less Number of Frames
NF = NFrames -1;

% Get Fg[k]'s 
T = FrameToFrameTransformation(gBar,gTilda,Gi,NF);

%% Run Pivot Calibration Method (ref. Slide 16 W11-L2)

% Init
p = zeros(3*(NFrames-1),1);
A = zeros(3*(NFrames-1),6);

% Parsing Tranformation Matrix 
for f = 1:(NFrames-1)
    % Grab Rotation Part of Tranformation Matrix Obtained Above
    RBuff = T(1:3,(f*4)-3:(f*4)-1);
    A((f*3)-2:(f*3),1:3) = RBuff;

    % Fill out the rest of A matrix
    A((f*3)-2:(f*3),4:6) = -eye(3);

    % Grab Translation Part of Tranformation Matrix Obtained Above
    pBuff = T(1:3,(f*4));
    p((f*3)-2:(f*3)) = -pBuff;

%     Ttemp((f*4)-3:(f*4),:) = [RBuff pBuff
%                               0 0 0 1];
end

% Least Squares
b = inv(A'*A)*A'*p;

% Tip Vector
bTip = b(1:3);

% Post Vector
bPost = b(4:6);

% bTip = Ttemp*bTip;


end



    