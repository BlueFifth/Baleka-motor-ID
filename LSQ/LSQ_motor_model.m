clear;
clc;
% Initial guesses 
% hold off
% Solve 1:
%%


% vel tests 11 & 15 are useless! bad choices - goes over max speed

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
index = 1;
qStepSize = Tests(1, index);
wStart = Tests(2, index);
wStepSize = Tests(3, index);
Kp = Tests(4, index);
Kd = Tests(5, index);
StepTime = 5;
% wants initial vals here too just to compile
% From velocity tests for M3:
J = 0.0210*100;
b = 0.0933*10;

% % From velocity tests for M2:
% J = 0.0175*100;
% b = 0.097*10;

% J = 1.592585250325430;
% b = 0.121282465151455; 

Bt = 0.5;
Ct = 0.001;
%%

% Import test data for comparison
TestsData1 = load("Data/StepTestsRound1V1.mat");
TestsData2 = load("Data/StepTestsRound2V1.mat");
MotorNum = 10; % Back left M3
% MotorNum = 9; % Front Right M5
% MotorNum = 8; % Front Left M6
% MotorNum = 7; % Back right M2


% Position tests start at 5s
% End varies on tests. For solver time reasons cal it at 10.5 (a bit longer
% than longest test) - might need to adjust
SimTimePos = 2.5;
stepsq = [];


temp = timeseries2timetable(TestsData1.data{index}{MotorNum}.Values.q);
Step1 = temp(5001:5001 + SimTimePos*1000,:);

temp = timeseries2timetable(TestsData2.data{index}{MotorNum}.Values.q);
Step2 = temp(5001:5001 + SimTimePos*1000,:);

%%

[J, b] = AK10lsqPosJb(MotorNum, J, b) % Quick start guess for j and b %[output:0575ce84] %[output:7805e7e3]
hold off
%%
[J, b, Bt, Ct] = AK10lsqPos(MotorNum, J, b, Bt, Ct, 3) % Refine using informed guess, limited time
hold off
clear startplot
%%

[J, b, Bt, Ct] = AK10lsqPos(MotorNum, J, b, Bt, Ct, 5.5) % Refine again with
hold off
%%
function [J, b] = AK10lsqPosJb(MotorNum, J0, b0)
%% LSQ for inertia (J), damping (b)


% Data trimming for lsq:
TestsData1 = load("Data/StepTestsRound1V1.mat");
TestsData2 = load("Data/StepTestsRound2V1.mat");
% Start with oscillatory first for best odds at global minima
% So reduce sample

% Lets use 1:7 for steps, 11:14 for validation
% Which motor? 
% Start with back left

% Position tests start at 5s
% End varies on tests. For solver time reasons cal it at 10.5 (a bit longer
% than longest test) - might need to adjust
SimTimePos = 1.5;
stepsq = [];
for i = 1:7
    temp = timeseries2timetable(TestsData1.data{i}{MotorNum}.Values.q);
    temp = temp.q(5001:5001 + SimTimePos*1000);
    stepsq = [stepsq; temp];
    temp = timeseries2timetable(TestsData2.data{i}{MotorNum}.Values.q);
    temp = temp.q(5001:5001 + SimTimePos*1000);
    stepsq = [stepsq; temp];
end
t = linspace(0, 14*(SimTimePos), 14*(SimTimePos)/1e-3+14);
mdl = 'AK109_LSQ_simpFric';
open_system(mdl); %Load the model
in = Simulink.SimulationInput(mdl); % Create simulation input object
in = in.setModelParameter("StopTime",num2str(SimTimePos));
in = in.setVariable('StepTime', 0, 'Workspace',mdl);

% Starting guess: From Sys ID toolbox
J = J0; % scaled by 100
b = b0; %scaled by 10 for unity solving 


pid0 = [J, b];

options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt',...
   'Display','iter-detailed');%, 'StepTolerance',0.001,'OptimalityTolerance',0.001);
% Optimize the gains
% set_param(mdl,'FastRestart','on');           % Fast restart


% % set_param(mdl,'FastRestart','on');
% Return the gains
% 
pid = lsqnonlin(@JBlsq,pid0,[],[], options);
% pid = lsqnonlin(@JBlsq,pid0,[0.01, 0.01],[10, 10], options);
J = pid(1);
b = pid(2);
    function F = JBlsq(pid)
      persistent startplot
       if isempty(startplot)
          figure
          plot(t, stepsq, 'r', 'DisplayName', 'Reference Step')
          legend(AutoUpdate="off")
          title("Attempts at solving for J, b")
          xlabel("Time (s)")
          ylabel("Angle (rad)")
          hold on
          startplot = true;
       end
      in = in.setVariable('J',pid(1),'Workspace',mdl);
      in = in.setVariable('b', pid(2), 'Workspace', mdl);

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
      simdata = [];
      for i=1:7
          in = in.setVariable('qStepSize', Tests(1, i));% , 'Workspace', mdl);
          in = in.setVariable('wStart', Tests(2,i));%;;%'Workspace', mdl);
          in = in.setVariable('wStepSize', Tests(3,i));%,;% 'Workspace', mdl);
          in = in.setVariable('Kp', Tests(4,i));%,;'Workspace', mdl);
          in = in.setVariable('Kd', Tests(5,i));% ;%'Workspace', mdl);
          out = sim(in);
          simdata =[simdata; out.yout{1}.Values.Data; out.yout{1}.Values.Data] ;
      end
          

      F = simdata - stepsq;
      plot(t, simdata, ':')
    end
end
%%
function [J, b, Bt, Ct] = AK10lsqPos(MotorNum, J0, b0, Bt0, Ct0, t_sim)

%% LSQ for inertia (J), damping (b) , Breakaway torque (Bt), and coloumb friction torque (Ct)


% Data trimming for lsq:
TestsData1 = load("Data/StepTestsRound1V1.mat");
TestsData2 = load("Data/StepTestsRound2V1.mat");
% Start with oscillatory first for best odds at global minima
% So reduce sample

% Lets use 1:7 for steps, 11:14 for validation
% Position tests start at 5s
% End varies on tests. For solver time reasons cal it at 10.5 (a bit longer
% than longest test) - might need to adjust
SimTimePos = t_sim;
stepsq = [];

for i = 1:7
    temp = timeseries2timetable(TestsData1.data{i}{MotorNum}.Values.q);
    temp = temp.q(5001:5001 + SimTimePos*1000);
    stepsq = [stepsq; temp];
    temp = timeseries2timetable(TestsData2.data{i}{MotorNum}.Values.q);
    temp = temp.q(5001:5001 + SimTimePos*1000);
    stepsq = [stepsq; temp];
end
t = linspace(0, 14*SimTimePos, 14*SimTimePos/1e-3+14);
mdl = 'AK109_LSQ';
open_system(mdl); %Load the model
in = Simulink.SimulationInput(mdl); % Create simulation input object
in = in.setModelParameter("StopTime",num2str(SimTimePos));
in = in.setVariable('StepTime', 0, 'Workspace',mdl);

% Starting guess
J = J0; % scaled by 100
b = b0; %scaled by 10 for unity solving 
Bt = Bt0; % Breakaway friction torque
Ct = Ct0; % Coulomb torque


pid0 = [J; b; Bt; Ct];

options = optimoptions(@lsqnonlin,'Algorithm','interior-point',...
   'Display','iter-detailed');%, 'StepTolerance',0.1,'OptimalityTolerance',0.001);
% Optimize the gains
% Make Ct < Bt
% Ct -Bt <0
% A = [-1 1];
A = [0, 0, -1, 1];
beq = 0.0;
Aeq = [];
Beq = [];
pid = lsqnonlin(@lsqJbBtCt,pid0,[0.1, 0.00001, 0.3, 0.00001],[5, 1, 1, 0.3],A, beq, Aeq, Beq, @nlcon, options);
% pid = lsqnonlin(@tracklsq,pid0,[0.01, 0.001],[1, 0.5],A, b, Aeq, Beq, @nlcon, options);
% pid = lsqnonlin(@tracklsq,pid0,[1.0, 0.5, 0.01],[1.7, 1, 0.5], options);
% set_param(mdl,'FastRestart','on');
% Return the gains
J = pid(1); b = pid(2); Bt = pid(3); Ct = pid(4);
    function F = lsqJbBtCt(pid)
      persistent startplot
       if isempty(startplot)
          figure
          plot(t, stepsq, 'r', 'DisplayName', 'Reference Step')
          legend(AutoUpdate="off")
          title("Attempts at solving for J, b, Bt, Ct")
          xlabel("Time (s)")
          ylabel("Angle (rad)")
          hold on
          startplot = true;
       end
      % Track the output of optsim to a signal of 1
      % Set the simulation input object parameters
      % in = in.setVariable('Ip',pid(1),'Workspace',mdl);
      % in = in.setVariable('Ii',pid(2),'Workspace',mdl);
      in = in.setVariable('J',pid(1),'Workspace',mdl);
      in = in.setVariable('b', pid(2), 'Workspace', mdl);
      in = in.setVariable('Bt',pid(3),'Workspace',mdl);
      in = in.setVariable('Ct', pid(4), 'Workspace', mdl);
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
      simdata = [];
      for i=1:7
          in = in.setVariable('qStepSize', Tests(1, i));% , 'Workspace', mdl);
          in = in.setVariable('wStart', Tests(2,i));%;;%'Workspace', mdl);
          in = in.setVariable('wStepSize', Tests(3,i));%,;% 'Workspace', mdl);
          in = in.setVariable('Kp', Tests(4,i));%,;'Workspace', mdl);
          in = in.setVariable('Kd', Tests(5,i));% ;%'Workspace', mdl);
          out = sim(in);
          simdata =[simdata; out.yout{1}.Values.Data; out.yout{1}.Values.Data] ;
      end

      F = simdata - stepsq;
      plot(t, simdata, ':')
    end

    function [c, ceq] = nlcon(x)
        ceq = [];
        c = [];
    end
end
%%

function [Ct] = AK10lsqVel

SimTime = 1.5;
% Get comparison data
data = cell(1,9);
load("VelocitySteps.mat");
steps = cell(1, 9);
for i=1:9
    steps{i} = timeseries2timetable(data{i}{4}.Values);
    steps{i} = steps{i}.M1_vel(4501:4501 + SimTime*1000);
end
stepq = [steps{3}; steps{4};steps{5};steps{6}; steps{9}];

mdl = 'AK109_simpFric';
open_system(mdl) %Load the model
in = Simulink.SimulationInput(mdl); % Create simulation input object
in = in.setModelParameter("StopTime",num2str(SimTime));
in = in.setVariable('StepTime', 0.5, 'Workspace',mdl);
in = in.setVariable('q', 0, 'Workspace', mdl);
in = in.setVariable('Kp', 0, 'Workspace', mdl);


J = 1.59;
b = 1.49;
Bt = 0.7; % Breakaway friction torque
Ct = 0.144; % Coulomb torque
pid0 = [Ct];

options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt',...
   'Display','iter-detailed');
% Optimize the gains
% set_param(mdl,'FastRestart','off');           % Fast restart
% Make Ct < Bt
% Ct -Bt <0
% A = [0 0 0 0 -1 1];
% A = [0 0 -1 1];
% b = 0;
% Aeq = [];
% Beq = [];
% pid = lsqnonlin(@tracklsq,pid0,[0.1, 0.1, 0.00001, 0.00001, .000001, .000001],[50, 50, 50, 10000, 50, 50],A, b, Aeq, Beq, @nlcon, options);
% pid = lsqnonlin(@tracklsq,pid0,[0.00001, 0.00001, .000001, .000001],[50, 10000, 5, 5],A, b, Aeq, Beq, @nlcon, options);
pid = lsqnonlin(@tracklsq,pid0,[0.001],[0.7], options);
set_param(mdl,'FastRestart','off');
% Return the gains
% J = pid(1); b = pid(2); Bt = pid(3); Ct = pid(4);
Ct = pid(1);
    function F = tracklsq(pid)
      persistent startplot
       if isempty(startplot)
          figure
          plot(stepq, 'r', 'DisplayName', 'Reference Step')
          legend(AutoUpdate="off")
          title("Attempts at solving")
          hold on
          startplot = true;
      end
      % Track the output of optsim to a signal of 1
      % Set the simulation input object parameters
      in = in.setVariable('Ct', pid(1), 'Workspace', mdl);

      
      % Step 1
      in = in.setVariable('StartVel', 1, 'Workspace', mdl);
      in = in.setVariable('StepVel', 30, 'Workspace', mdl);
      in = in.setVariable('Kd', 1, 'Workspace', mdl);
      out = sim(in);
      simdata = out.yout{1}.Values.Data;
      
      % Step 2
      in = in.setVariable('StartVel', 1, 'Workspace', mdl);
      in = in.setVariable('StepVel', -20, 'Workspace', mdl);
      in = in.setVariable('Kd', 1, 'Workspace', mdl);
      out = sim(in);
      simdata = [simdata; out.yout{1}.Values.Data];

      % Step 3
      in = in.setVariable('StartVel', 5, 'Workspace', mdl);
      in = in.setVariable('StepVel', 20, 'Workspace', mdl);
      in = in.setVariable('Kd', 1, 'Workspace', mdl);
      out = sim(in);
      simdata = [simdata; out.yout{1}.Values.Data];

      % Step 4
      in = in.setVariable('StartVel', -5, 'Workspace', mdl);
      in = in.setVariable('StepVel', 15, 'Workspace', mdl);
      in = in.setVariable('Kd', 1, 'Workspace', mdl);
      out = sim(in);
      simdata = [simdata; out.yout{1}.Values.Data];

      % Step 5
      in = in.setVariable('StartVel', -2, 'Workspace', mdl);
      in = in.setVariable('StepVel', -30, 'Workspace', mdl);
      in = in.setVariable('Kd', 5, 'Workspace', mdl);
      out = sim(in);
      simdata = [simdata; out.yout{1}.Values.Data];

      F = simdata - stepq;
      plot(simdata, ':')
    end

    function [c, ceq] = nlcon(x)
        ceq = [];
        c = [];
    end
end

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":44.2}
%---
%[output:0575ce84]
%   data: {"dataType":"text","outputData":{"text":"\n                                        First-order                     Norm of \n Iteration  Func-count      Resnorm      optimality       Lambda           step\n     0           3          14190.2         4.6e+03         0.01\n     1          12          8193.19        7.89e+03        10000       0.432817\n     2          15          2524.06        7.31e+03         1000       0.382978\n     3          18          1069.49        4.12e+03          100       0.215214\n","truncated":false}}
%---
%[output:7805e7e3]
%   data: {"dataType":"error","outputData":{"errorType":"runtime","text":"Error using <a href=\"matlab:matlab.lang.internal.introspective.errorDocCallback('LSQ_motor_model>AK10lsqPosJb\/JBlsq', 'C:\\Users\\Gavin\\Documents\\GitHub\\Baleka-motor-ID\\LSQ\\LSQ_motor_model.m', 169)\" style=\"font-weight:bold\">LSQ_motor_model>AK10lsqPosJb\/JBlsq<\/a> (<a href=\"matlab: opentoline('C:\\Users\\Gavin\\Documents\\GitHub\\Baleka-motor-ID\\LSQ\\LSQ_motor_model.m',169,0)\">line 169<\/a>)\nProgram interruption has been detected.\n\nError in finDiffEvalAndChkErr\n\nError in finitedifferences\n\nError in computeFinDiffGradAndJac\n\nError in <a href=\"matlab:matlab.lang.internal.introspective.errorDocCallback('levenbergMarquardt', 'C:\\Program Files\\MATLAB\\R2025a\\toolbox\\shared\\optimlib\\levenbergMarquardt.m', 299)\" style=\"font-weight:bold\">levenbergMarquardt<\/a> (<a href=\"matlab: opentoline('C:\\Program Files\\MATLAB\\R2025a\\toolbox\\shared\\optimlib\\levenbergMarquardt.m',299,0)\">line 299<\/a>)\n            [JAC,~,~,numEvals,evalOK] = computeFinDiffGradAndJac(x,objfun,[],costFun, ...\n            ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\nError in <a href=\"matlab:matlab.lang.internal.introspective.errorDocCallback('lsqncommon', 'C:\\Program Files\\MATLAB\\R2025a\\toolbox\\shared\\optimlib\\lsqncommon.m', 188)\" style=\"font-weight:bold\">lsqncommon<\/a> (<a href=\"matlab: opentoline('C:\\Program Files\\MATLAB\\R2025a\\toolbox\\shared\\optimlib\\lsqncommon.m',188,0)\">line 188<\/a>)\n        levenbergMarquardt(funfcn,xC,lb,ub,flags.verbosity,options,F,Jac, ...\n        ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\nError in <a href=\"matlab:matlab.lang.internal.introspective.errorDocCallback('lsqnonlin', 'C:\\Program Files\\MATLAB\\R2025a\\toolbox\\shared\\optimlib\\lsqnonlin.m', 264)\" style=\"font-weight:bold\">lsqnonlin<\/a> (<a href=\"matlab: opentoline('C:\\Program Files\\MATLAB\\R2025a\\toolbox\\shared\\optimlib\\lsqnonlin.m',264,0)\">line 264<\/a>)\n    lsqncommon(funfcn,xCurrent,lb,ub,options,defaultopt,caller,...\n    ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\nError in <a href=\"matlab:matlab.lang.internal.introspective.errorDocCallback('LSQ_motor_model>AK10lsqPosJb', 'C:\\Users\\Gavin\\Documents\\GitHub\\Baleka-motor-ID\\LSQ\\LSQ_motor_model.m', 131)\" style=\"font-weight:bold\">LSQ_motor_model>AK10lsqPosJb<\/a> (<a href=\"matlab: opentoline('C:\\Users\\Gavin\\Documents\\GitHub\\Baleka-motor-ID\\LSQ\\LSQ_motor_model.m',131,0)\">line 131<\/a>)\npid = lsqnonlin(@JBlsq,pid0,[],[], options);\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^"}}
%---
