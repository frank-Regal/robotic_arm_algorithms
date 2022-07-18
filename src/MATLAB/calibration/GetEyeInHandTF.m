%% Title:    THA3, Get Eye In Hand Transformation (PA2)
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2022.04.29
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

function [Tx]=GetEyeInHandTF(q_Robot_config, q_camera_config,t_Robot_config,t_camera_config)

% Grab number of rows from robot data (assumption is this is consistent
% across all input files)
[rows,~] = size(q_Robot_config);

% Init M
M = zeros((rows*4)-4,4);
RA = zeros((rows*3)-3,3);
RB = zeros((rows*3)-3,3);
ta = zeros(rows-1,3);
tb = zeros(rows-1,3);

% Calculate the First Inverse Tranformation Matrix of E1
RE1 = QuaternionToRot(q_Robot_config(1,:));
TE1 = t_Robot_config(1,:);
E1 = [RE1 TE1';
    zeros(1,3) 1];
Inv_E1 = InverseTransformMatrix(E1);

% Calculate the First Inverse Tranformation Matrix of S1
RS1 = QuaternionToRot(q_camera_config(1,:));
TS1 = t_camera_config(1,:);
S1 = [RS1 TS1';
    zeros(1,3) 1];
Inv_S1 = InverseTransformMatrix(S1);

% Loop Through Data Set to Build M matrix
for i = 1:rows-1

    % Build A Transformation Matrix from two Consecutive Quaternions
    REK = QuaternionToRot(q_Robot_config(i+1,:));
    TEK = t_Robot_config(i+1,:);
    EK = [REK TEK';
          zeros(1,3) 1];
    A = EK * Inv_E1;
    RA(((i*3)-2):(i*3),:) = A(1:3,1:3);

    % Build B Transformation Matrix from two Consecutive Quaternions
    RSK = QuaternionToRot(q_camera_config(i+1,:));
    TSK = t_camera_config(i+1,:);
    SK = [RSK TSK';
          zeros(1,3) 1];
    B = SK*Inv_S1;
    RB(((i*3)-2):(i*3),:) = B(1:3,1:3);

    % Convert A and B to quaternions to Find X tranform using AX=XB
    qa = RotToQuaternion(A(1:3,1:3));
    ta(i,:) = A(1:3,4)';

    qb = RotToQuaternion(B(1:3,1:3));
    tb(i,:) = B(1:3,4)';

    % Build M Matrix
    elm11 = qa(1) - qb(1);
    elm21 = qa(2:4)' - qb(2:4)';
    elm12 = -(qa(2:4)' - qb(2:4)')';
    elm22 = elm11*eye(3) + VectorToSkewMatrix(qa(2:4)'+qb(2:4)');
    M((i*4)-3:(i*4),:) = [elm11 elm12;
                          elm21 elm22]
end 

% Solve for Rx using SVD
[~,~,V] = svd(M);

% Fourth column is resulting quaternion by derivation presented in class
V4 = V(:,4);

% Convert To Rotation from Quaternion
RX = QuaternionToRot(V4);

% init translation vector
px = zeros(rows-1,3);

% Solve for px
for j = i:rows-1
    
    % Setup RHS
    b = RX * tb(j,:)' - ta(j,:)';
    
    % Setup LHS
    A = RA(((j*3)-2):(j*3),:) - eye(3);
    
    % Use built in least squares solver 
    px(j,:) = lsqr(A,b);
end

% Take Average of translation vectors found from least squares solvers
min_px_x = mean(px(:,1));
min_px_y = mean(px(:,2));
min_px_z = mean(px(:,3));
px_final = [min_px_x;
            min_px_y;
            min_px_z];

% Output Transform
Tx = [RX px_final;
      0 0 0 1];

end