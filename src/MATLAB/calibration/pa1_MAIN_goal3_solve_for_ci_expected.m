%% Title:    THA3, Test Functions for Ci Expected Calculations
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2022.04.29
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

close all;
clear;
clc;

fprintf("\n\n\n");
disp("////////////////////////////////////////////////////////////////");

%% // Data Set A
% Get Parsed Data of Calibration Body File
[~, dBar, dTilda, ci, ~, ~, ~, aBar, aTilda]=ReadCalBody('pa1-debug-a-calbody.txt');

% Get Parsed Data of Calibration Readings
[Di, Ai, ~, NumFrames]=ReadCalReadings('pa1-debug-a-calreadings.txt');

% Calculate Ta, a fat matrix consisting of one Fa for each frame
Ta = FrameToFrameTransformation(aBar,aTilda,Ai,NumFrames);

% Calculate Td, a fat matrix consisting of one Fd for each frame
Td = FrameToFrameTransformation(dBar,dTilda,Di,NumFrames);

% Calculate Ci Expected
CiExpected = CalcCiExpected(Ta,Td,ci,NumFrames);

% Print Out
fprintf("\n\nData Set A: Ci Expected Results:\n")
disp(CiExpected);

%% // Data Set B
disp("////////////////////////////////////////////////////////////////");
% Get Parsed Data of Calibration Body File
[~, dBar, dTilda, ci, ~, ~, ~, aBar, aTilda]=ReadCalBody('pa1-debug-b-calbody.txt');

% Get Parsed Data of Calibration Readings
[Di, Ai, ~, NumFrames]=ReadCalReadings('pa1-debug-b-calreadings.txt');

% Calculate Ta, a fat matrix consisting of one Fa for each frame
Ta = FrameToFrameTransformation(aBar,aTilda,Ai,NumFrames);

% Calculate Td, a fat matrix consisting of one Fd for each frame
Td = FrameToFrameTransformation(dBar,dTilda,Di,NumFrames);

% Calculate Ci Expected
CiExpected = CalcCiExpected(Ta,Td,ci,NumFrames);

% Print Out
fprintf("\n\nData Set B: Ci Expected Results:\n")
disp(CiExpected);

%% // Data Set C
disp("////////////////////////////////////////////////////////////////");
% Get Parsed Data of Calibration Body File
[~, dBar, dTilda, ci, ~, ~, ~, aBar, aTilda]=ReadCalBody('pa1-debug-c-calbody.txt');

% Get Parsed Data of Calibration Readings
[Di, Ai, ~, NumFrames]=ReadCalReadings('pa1-debug-c-calreadings.txt');

% Calculate Ta, a fat matrix consisting of one Fa for each frame
Ta = FrameToFrameTransformation(aBar,aTilda,Ai,NumFrames);

% Calculate Td, a fat matrix consisting of one Fd for each frame
Td = FrameToFrameTransformation(dBar,dTilda,Di,NumFrames);

% Calculate Ci Expected
CiExpected = CalcCiExpected(Ta,Td,ci,NumFrames);

% Print Out
fprintf("\n\nData Set C: Ci Expected Results:\n")
disp(CiExpected);

%% // Data Set D
disp("////////////////////////////////////////////////////////////////");
% Get Parsed Data of Calibration Body File
[~, dBar, dTilda, ci, ~, ~, ~, aBar, aTilda]=ReadCalBody('pa1-debug-d-calbody.txt');

% Get Parsed Data of Calibration Readings
[Di, Ai, ~, NumFrames]=ReadCalReadings('pa1-debug-d-calreadings.txt');

% Calculate Ta, a fat matrix consisting of one Fa for each frame
Ta = FrameToFrameTransformation(aBar,aTilda,Ai,NumFrames);

% Calculate Td, a fat matrix consisting of one Fd for each frame
Td = FrameToFrameTransformation(dBar,dTilda,Di,NumFrames);

% Calculate Ci Expected
CiExpected = CalcCiExpected(Ta,Td,ci,NumFrames);

% Print Out
fprintf("\n\nData Set D: Ci Expected Results:\n")
disp(CiExpected);

%% // Data Set E
disp("////////////////////////////////////////////////////////////////");
% Get Parsed Data of Calibration Body File
[~, dBar, dTilda, ci, ~, ~, ~, aBar, aTilda]=ReadCalBody('pa1-debug-e-calbody.txt');

% Get Parsed Data of Calibration Readings
[Di, Ai, ~, NumFrames]=ReadCalReadings('pa1-debug-e-calreadings.txt');

% Calculate Ta, a fat matrix consisting of one Fa for each frame
Ta = FrameToFrameTransformation(aBar,aTilda,Ai,NumFrames);

% Calculate Td, a fat matrix consisting of one Fd for each frame
Td = FrameToFrameTransformation(dBar,dTilda,Di,NumFrames);

% Calculate Ci Expected
CiExpected = CalcCiExpected(Ta,Td,ci,NumFrames);

% Print Out
fprintf("\n\nData Set E: Ci Expected Results:\n")
disp(CiExpected);

%% // Data Set F
disp("////////////////////////////////////////////////////////////////");
% Get Parsed Data of Calibration Body File
[~, dBar, dTilda, ci, ~, ~, ~, aBar, aTilda]=ReadCalBody('pa1-debug-f-calbody.txt');

% Get Parsed Data of Calibration Readings
[Di, Ai, ~, NumFrames]=ReadCalReadings('pa1-debug-f-calreadings.txt');

% Calculate Ta, a fat matrix consisting of one Fa for each frame
Ta = FrameToFrameTransformation(aBar,aTilda,Ai,NumFrames);

% Calculate Td, a fat matrix consisting of one Fd for each frame
Td = FrameToFrameTransformation(dBar,dTilda,Di,NumFrames);

% Calculate Ci Expected
CiExpected = CalcCiExpected(Ta,Td,ci,NumFrames);

% Print Out
fprintf("\n\nData Set F: Ci Expected Results:\n")
disp(CiExpected);

%% // Data Set G
disp("////////////////////////////////////////////////////////////////");
% Get Parsed Data of Calibration Body File
[~, dBar, dTilda, ci, ~, ~, ~, aBar, aTilda]=ReadCalBody('pa1-debug-g-calbody.txt');

% Get Parsed Data of Calibration Readings
[Di, Ai, ~, NumFrames]=ReadCalReadings('pa1-debug-g-calreadings.txt');

% Calculate Ta, a fat matrix consisting of one Fa for each frame
Ta = FrameToFrameTransformation(aBar,aTilda,Ai,NumFrames);

% Calculate Td, a fat matrix consisting of one Fd for each frame
Td = FrameToFrameTransformation(dBar,dTilda,Di,NumFrames);

% Calculate Ci Expected
CiExpected = CalcCiExpected(Ta,Td,ci,NumFrames);

% Print Out
fprintf("\n\nData Set G: Ci Expected Results:\n")
disp(CiExpected);

%% // Data Set H
disp("////////////////////////////////////////////////////////////////");
% Get Parsed Data of Calibration Body File
[~, dBar, dTilda, ci, ~, ~, ~, aBar, aTilda]=ReadCalBody('pa1-unknown-h-calbody.txt');

% Get Parsed Data of Calibration Readings
[Di, Ai, ~, NumFrames]=ReadCalReadings('pa1-unknown-h-calreadings.txt');

% Calculate Ta, a fat matrix consisting of one Fa for each frame
Ta = FrameToFrameTransformation(aBar,aTilda,Ai,NumFrames);

% Calculate Td, a fat matrix consisting of one Fd for each frame
Td = FrameToFrameTransformation(dBar,dTilda,Di,NumFrames);

% Calculate Ci Expected
CiExpected = CalcCiExpected(Ta,Td,ci,NumFrames);

% Print Out
fprintf("\n\nData Set H: Ci Expected Results:\n")
disp(CiExpected);

%% // Data Set I
disp("////////////////////////////////////////////////////////////////");
% Get Parsed Data of Calibration Body File
[~, dBar, dTilda, ci, ~, ~, ~, aBar, aTilda]=ReadCalBody('pa1-unknown-i-calbody.txt');

% Get Parsed Data of Calibration Readings
[Di, Ai, ~, NumFrames]=ReadCalReadings('pa1-unknown-i-calreadings.txt');

% Calculate Ta, a fat matrix consisting of one Fa for each frame
Ta = FrameToFrameTransformation(aBar,aTilda,Ai,NumFrames);

% Calculate Td, a fat matrix consisting of one Fd for each frame
Td = FrameToFrameTransformation(dBar,dTilda,Di,NumFrames);

% Calculate Ci Expected
CiExpected = CalcCiExpected(Ta,Td,ci,NumFrames);

% Print Out
fprintf("\n\nData Set I: Ci Expected Results:\n")
disp(CiExpected);

%% // Data Set J
disp("////////////////////////////////////////////////////////////////");
% Get Parsed Data of Calibration Body File
[~, dBar, dTilda, ci, ~, ~, ~, aBar, aTilda]=ReadCalBody('pa1-unknown-j-calbody.txt');

% Get Parsed Data of Calibration Readings
[Di, Ai, ~, NumFrames]=ReadCalReadings('pa1-unknown-j-calreadings.txt');

% Calculate Ta, a fat matrix consisting of one Fa for each frame
Ta = FrameToFrameTransformation(aBar,aTilda,Ai,NumFrames);

% Calculate Td, a fat matrix consisting of one Fd for each frame
Td = FrameToFrameTransformation(dBar,dTilda,Di,NumFrames);

% Calculate Ci Expected
CiExpected = CalcCiExpected(Ta,Td,ci,NumFrames);

% Print Out
fprintf("\n\nData Set J: Ci Expected Results:\n")
disp(CiExpected);

%% // Data Set K
disp("////////////////////////////////////////////////////////////////");
% Get Parsed Data of Calibration Body File
[~, dBar, dTilda, ci, ~, ~, ~, aBar, aTilda]=ReadCalBody('pa1-unknown-k-calbody.txt');

% Get Parsed Data of Calibration Readings
[Di, Ai, ~, NumFrames]=ReadCalReadings('pa1-unknown-k-calreadings.txt');

% Calculate Ta, a fat matrix consisting of one Fa for each frame
Ta = FrameToFrameTransformation(aBar,aTilda,Ai,NumFrames);

% Calculate Td, a fat matrix consisting of one Fd for each frame
Td = FrameToFrameTransformation(dBar,dTilda,Di,NumFrames);

% Calculate Ci Expected
CiExpected = CalcCiExpected(Ta,Td,ci,NumFrames);

% Print Out
fprintf("\n\nData Set K: Ci Expected Results:\n")
disp(CiExpected);
