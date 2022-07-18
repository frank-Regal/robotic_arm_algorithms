%% Title:    THA3, Test Functions for Optical tracker Pivot Calibrations
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2022.04.29
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

close all;
clear;
clc;

fprintf("\n\n\n");
%% Data Set A
disp("////////////////////////////////////////////////////////////////");
[Di,Hi,NumFrames] = ReadOptPivot('pa1-debug-a-optpivot.txt');
[~, dBar, dTilda, ~, ~, ~, ~, ~, ~]=ReadCalBody('pa1-debug-a-calbody.txt');
[bTip,bPost] = OptPivotCalibration(dBar,dTilda,Di,Hi,NumFrames);

fprintf("\n\nData Set A: Btip and Bpost Results:\n")
disp(bTip);
disp(bPost);

%% Data Set B
disp("////////////////////////////////////////////////////////////////");
[Di,Hi,NumFrames] = ReadOptPivot('pa1-debug-b-optpivot.txt');
[~, dBar, dTilda, ~, ~, ~, ~, ~, ~]=ReadCalBody('pa1-debug-b-calbody.txt');
[bTip,bPost] = OptPivotCalibration(dBar,dTilda,Di,Hi,NumFrames);


fprintf("\n\nData Set B: Btip and Bpost Results:\n")
disp(bTip);
disp(bPost);

%% Data Set C
disp("////////////////////////////////////////////////////////////////");
[Di,Hi,NumFrames] = ReadOptPivot('pa1-debug-c-optpivot.txt');
[~, dBar, dTilda, ~, ~, ~, ~, ~, ~]=ReadCalBody('pa1-debug-c-calbody.txt');
[bTip,bPost] = OptPivotCalibration(dBar,dTilda,Di,Hi,NumFrames);


fprintf("\n\nData Set C: Btip and Bpost Results:\n")
disp(bTip);
disp(bPost);

%% Data Set D
disp("////////////////////////////////////////////////////////////////");
[Di,Hi,NumFrames] = ReadOptPivot('pa1-debug-d-optpivot.txt');
[~, dBar, dTilda, ~, ~, ~, ~, ~, ~]=ReadCalBody('pa1-debug-d-calbody.txt');
[bTip,bPost] = OptPivotCalibration(dBar,dTilda,Di,Hi,NumFrames);


fprintf("\n\nData Set D: Btip and Bpost Results:\n")
disp(bTip);
disp(bPost);

%% Data Set E
disp("////////////////////////////////////////////////////////////////");
[Di,Hi,NumFrames] = ReadOptPivot('pa1-debug-e-optpivot.txt');
[~, dBar, dTilda, ~, ~, ~, ~, ~, ~]=ReadCalBody('pa1-debug-e-calbody.txt');
[bTip,bPost] = OptPivotCalibration(dBar,dTilda,Di,Hi,NumFrames);

fprintf("\n\nData Set E: Btip and Bpost Results:\n")
disp(bTip);
disp(bPost);

%% Data Set F
disp("////////////////////////////////////////////////////////////////");
[Di,Hi,NumFrames] = ReadOptPivot('pa1-debug-f-optpivot.txt');
[~, dBar, dTilda, ~, ~, ~, ~, ~, ~]=ReadCalBody('pa1-debug-f-calbody.txt');
[bTip,bPost] = OptPivotCalibration(dBar,dTilda,Di,Hi,NumFrames);

fprintf("\n\nData Set F: Btip and Bpost Results:\n")
disp(bTip);
disp(bPost);

%% Data Set G
disp("////////////////////////////////////////////////////////////////");
[Di,Hi,NumFrames] = ReadOptPivot('pa1-debug-g-optpivot.txt');
[~, dBar, dTilda, ~, ~, ~, ~, ~, ~]=ReadCalBody('pa1-debug-g-calbody.txt');
[bTip,bPost] = OptPivotCalibration(dBar,dTilda,Di,Hi,NumFrames);

fprintf("\n\nData Set G: Btip and Bpost Results:\n")
disp(bTip);
disp(bPost);

%% Data Set H
disp("////////////////////////////////////////////////////////////////");
[Di,Hi,NumFrames] = ReadOptPivot('pa1-unknown-h-optpivot.txt');
[~, dBar, dTilda, ~, ~, ~, ~, ~, ~]=ReadCalBody('pa1-unknown-h-calbody.txt');
[bTip,bPost] = OptPivotCalibration(dBar,dTilda,Di,Hi,NumFrames);

fprintf("\n\nData Set H: Btip and Bpost Results:\n")
disp(bTip);
disp(bPost);

%% Data Set I
disp("////////////////////////////////////////////////////////////////");
[Di,Hi,NumFrames] = ReadOptPivot('pa1-unknown-i-optpivot.txt');
[~, dBar, dTilda, ~, ~, ~, ~, ~, ~]=ReadCalBody('pa1-unknown-i-calbody.txt');
[bTip,bPost] = OptPivotCalibration(dBar,dTilda,Di,Hi,NumFrames);

fprintf("\n\nData Set I: Btip and Bpost Results:\n")
disp(bTip);
disp(bPost);

%% Data Set J
disp("////////////////////////////////////////////////////////////////");
[Di,Hi,NumFrames] = ReadOptPivot('pa1-unknown-j-optpivot.txt');
[~, dBar, dTilda, ~, ~, ~, ~, ~, ~]=ReadCalBody('pa1-unknown-j-calbody.txt');
[bTip,bPost] = OptPivotCalibration(dBar,dTilda,Di,Hi,NumFrames);

fprintf("\n\nData Set J: Btip and Bpost Results:\n")
disp(bTip);
disp(bPost);

%% Data Set K
disp("////////////////////////////////////////////////////////////////");
[Di,Hi,NumFrames] = ReadOptPivot('pa1-unknown-k-optpivot.txt');
[~, dBar, dTilda, ~, ~, ~, ~, ~, ~]=ReadCalBody('pa1-unknown-k-calbody.txt');
[bTip,bPost] = OptPivotCalibration(dBar,dTilda,Di,Hi,NumFrames);

fprintf("\n\nData Set K: Btip and Bpost Results:\n")
disp(bTip);
disp(bPost);