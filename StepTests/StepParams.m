clear;
clc;

%%
%[text] ## Motor IDs:
% Update these!
M_Front_left = 2;
M_FL_torque = 1;

M_Front_right = 3;
M_FR_torque = 1;

M_Back_left = 6;
M_BL_torque = 1;

M_Back_right = 5;
M_BR_torque = 1;
%%
%[text] ## Step Params
I = 1; % For ease of swapping tests
% Big array for portability
% QstepSize
% wStart
% wStepSize
% Kp
% Kd
% Test number:  1  2  3  4  5  6  7  8  9 10 11  12  13  14  15          
Tests = [       3, 5, 1,-1,-4,-1, 6, 3,-2, 2, 0,  0,  0,  0,  0;
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1,  1,  5, -5, -2;
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0,30,-20, 20, 15,-30;
                1, 1, 1, 5, 5, 5, 5, 2, 2, 3, 0,  0,  0,  0,  0;
                0, 0, 0, 0, 1, 1, 1, 0, 1, 0, 1,  1,  1,  1,  2];
qStepSize = Tests(1, I);
wStart=Tests(2, I);
wStepSize = Tests(3, I);
Kp = Tests(4, I);
Kd = Tests(5, I);
StopTime=15;


%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":40}
%---
