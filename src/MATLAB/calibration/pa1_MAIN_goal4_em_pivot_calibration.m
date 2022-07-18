%% Title:    THA3, Test Functions for Em Pivot Calibrations
% Course:    ME397 Algorithms for Sensor Based Robots
% Professor: Dr. Alambeigi
% Due Date:  2022.04.29
% School:    The University of Texas at Austin
% Authors:   Frank Regal & Will Hancock

close all;
clc;
clear;

fprintf("\n\n\n");

%% Data Set A
disp("////////////////////////////////////////////////////////////////");
[GJ,NumFrames] = ReadEmPivot('pa1-debug-a-empivot.txt');
[bTip,bPost] = EmPivotCalibration(GJ,NumFrames);

fprintf("\n\nData Set A: Btip and Bpost Results:\n")
disp(bTip);
disp(bPost);

%% Data Set B
disp("////////////////////////////////////////////////////////////////");
[GJ,NumFrames] = ReadEmPivot('pa1-debug-b-empivot.txt');
[bTip,bPost] = EmPivotCalibration(GJ,NumFrames);


fprintf("\n\nData Set B: Btip and Bpost Results:\n")
disp(bTip);
disp(bPost);

%% Data Set C
disp("////////////////////////////////////////////////////////////////");
[GJ,NumFrames] = ReadEmPivot('pa1-debug-c-empivot.txt');
[bTip,bPost] = EmPivotCalibration(GJ,NumFrames);


fprintf("\n\nData Set C: Btip and Bpost Results:\n")
disp(bTip);
disp(bPost);

%% Data Set D
disp("////////////////////////////////////////////////////////////////");
[GJ,NumFrames] = ReadEmPivot('pa1-debug-d-empivot.txt');
[bTip,bPost] = EmPivotCalibration(GJ,NumFrames);


fprintf("\n\nData Set D: Btip and Bpost Results:\n")
disp(bTip);
disp(bPost);

%% Data Set E
disp("////////////////////////////////////////////////////////////////");
[GJ,NumFrames] = ReadEmPivot('pa1-debug-e-empivot.txt');
[bTip,bPost] = EmPivotCalibration(GJ,NumFrames);

fprintf("\n\nData Set E: Btip and Bpost Results:\n")
disp(bTip);
disp(bPost);

%% Data Set F
disp("////////////////////////////////////////////////////////////////");
[GJ,NumFrames] = ReadEmPivot('pa1-debug-f-empivot.txt');
[bTip,bPost] = EmPivotCalibration(GJ,NumFrames);

fprintf("\n\nData Set F: Btip and Bpost Results:\n")
disp(bTip);
disp(bPost);

%% Data Set G
disp("////////////////////////////////////////////////////////////////");
[GJ,NumFrames] = ReadEmPivot('pa1-debug-g-empivot.txt');
[bTip,bPost] = EmPivotCalibration(GJ,NumFrames);

fprintf("\n\nData Set G: Btip and Bpost Results:\n")
disp(bTip);
disp(bPost);

%% Data Set H
disp("////////////////////////////////////////////////////////////////");
[GJ,NumFrames] = ReadEmPivot('pa1-unknown-h-empivot.txt');
[bTip,bPost] = EmPivotCalibration(GJ,NumFrames);

fprintf("\n\nData Set H: Btip and Bpost Results:\n")
disp(bTip);
disp(bPost);

%% Data Set I
disp("////////////////////////////////////////////////////////////////");
[GJ,NumFrames] = ReadEmPivot('pa1-unknown-i-empivot.txt');
[bTip,bPost] = EmPivotCalibration(GJ,NumFrames);

fprintf("\n\nData Set I: Btip and Bpost Results:\n")
disp(bTip);
disp(bPost);

%% Data Set J
disp("////////////////////////////////////////////////////////////////");
[GJ,NumFrames] = ReadEmPivot('pa1-unknown-j-empivot.txt');
[bTip,bPost] = EmPivotCalibration(GJ,NumFrames);

fprintf("\n\nData Set J: Btip and Bpost Results:\n")
disp(bTip);
disp(bPost);

%% Data Set K
disp("////////////////////////////////////////////////////////////////");
[GJ,NumFrames] = ReadEmPivot('pa1-unknown-k-empivot.txt');
[bTip,bPost] = EmPivotCalibration(GJ,NumFrames);

fprintf("\n\nData Set K: Btip and Bpost Results:\n")
disp(bTip);
disp(bPost);