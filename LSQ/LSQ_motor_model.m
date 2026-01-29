clear;
clc;
% Initial guesses 
% hold off
% Solve 1:
%%




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
wStart = Tests(1, index);
wStepSize = Tests(1, index);
Kp = Tests(1, index);
Kd = Tests(1, index);
StepTime = Tests(1, index);
% wants initial vals here too
J = 1; % scaled by 100
b = 01; %scaled by 10 for unity solving 
%%
% [J, b, Bt, Ct] = AK10lsqPos
[J, b] = AK10lsqPosJb %[output:577f2787] %[output:745d7922] %[output:664b6a11] %[output:19e66aed]
%%
function [J, b, Bt, Ct] = AK10lsqPos

%% LSQ for inertia (J), damping (b) , Breakaway torque (Bt), and coloumb friction torque (Ct)


% Data trimming for lsq:
TestsData1 = load("Data/StepTestsRound1V1.mat");
TestsData2 = load("Data/StepTestsRound2V1.mat");
% Start with oscillatory first for best odds at global minima
% So reduce sample

% Lets use 1:7 for steps, 11:14 for validation
% Which motor? 
% Start with back left
MotorNum = 10; % Back left
% Position tests start at 5s
% End varies on tests. For solver time reasons cal it at 10.5 (a bit longer
% than longest test) - might need to adjust
SimTimePos = 2.5;
stepsq = [];

for i = 1:2
    temp = timeseries2timetable(TestsData1.data{i}{MotorNum}.Values.q);
    temp = temp.q(5001:5001 + SimTimePos*1000);
    stepsq = [stepsq; temp];
    temp = timeseries2timetable(TestsData2.data{i}{MotorNum}.Values.q);
    temp = temp.q(5001:5001 + SimTimePos*1000);
    stepsq = [stepsq; temp];
end

mdl = 'AK109_LSQ';
open_system(mdl) %Load the model
in = Simulink.SimulationInput(mdl); % Create simulation input object
in = in.setModelParameter("StopTime",num2str(SimTimePos));
in = in.setVariable('StepTime', 0, 'Workspace',mdl);

% Starting guess: From Sys ID toolbox
J = 2.10; % scaled by 100
b = 0.933; %scaled by 10 for unity solving 
Bt = 0.0; % Breakaway friction torque
Ct = 0.0; % Coulomb torque


pid0 = [J, b, Bt, Ct];

options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt',...
   'Display','iter-detailed');
% Optimize the gains
% Make Ct < Bt
% Ct -Bt <0
% A = [-1 1];
A = [0 0 -1 1];
b = 0;
Aeq = [];
Beq = [];
pid = lsqnonlin(@tracklsq,pid0,[0.1, 0.1, 0.00001, 0.00001, .000001, .000001],[50, 50, 50, 10000, 50, 50],A, b, Aeq, Beq, @nlcon, options);
% pid = lsqnonlin(@tracklsq,pid0,[0.01, 0.001],[1, 0.5],A, b, Aeq, Beq, @nlcon, options);
% pid = lsqnonlin(@tracklsq,pid0,[1.0, 0.5, 0.01],[1.7, 1, 0.5], options);
% set_param(mdl,'FastRestart','on');
% Return the gains
J = pid(1); b = pid(2); Bt = pid(3); Ct = pid(4);
    function F = tracklsq(pid)
      persistent startplot
       if isempty(startplot)
          figure
          plot(stepsq, 'r', 'DisplayName', 'Reference Step')
          legend(AutoUpdate="off")
          title("Attempts at solving")
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
      for i=1:2
          in = in.setVariable('qStepSize', Tests(1, i));% , 'Workspace', mdl);
          in = in.setVariable('wStart', Tests(2,i));%;;%'Workspace', mdl);
          in = in.setVariable('wStepSize', Tests(3,i));%,;% 'Workspace', mdl);
          in = in.setVariable('Kp', Tests(4,i));%,;'Workspace', mdl);
          in = in.setVariable('Kd', Tests(5,i));% ;%'Workspace', mdl);
          out = sim(in);
          simdata =[simdata; out.yout{1}.Values.Data; out.yout{1}.Values.Data] ;
      end

      F = simdata - stepsq;
      plot(simdata, ':')
    end

    function [c, ceq] = nlcon(x)
        ceq = [];
        c = [];
    end
end

%%
function [J, b] = AK10lsqPosJb

%% LSQ for inertia (J), damping (b)


% Data trimming for lsq:
TestsData1 = load("Data/StepTestsRound1V1.mat");
TestsData2 = load("Data/StepTestsRound2V1.mat");
% Start with oscillatory first for best odds at global minima
% So reduce sample

% Lets use 1:7 for steps, 11:14 for validation
% Which motor? 
% Start with back left
MotorNum = 10; % Back left
% Position tests start at 5s
% End varies on tests. For solver time reasons cal it at 10.5 (a bit longer
% than longest test) - might need to adjust
SimTimePos = 2.5;
stepsq = [];

for i = 1:2
    temp = timeseries2timetable(TestsData1.data{i}{MotorNum}.Values.q);
    temp = temp.q(5001:5001 + SimTimePos*1000);
    stepsq = [stepsq; temp];
    temp = timeseries2timetable(TestsData2.data{i}{MotorNum}.Values.q);
    temp = temp.q(5001:5001 + SimTimePos*1000);
    stepsq = [stepsq; temp];
end

mdl = 'AK109_LSQ_simpFric';
open_system(mdl) %Load the model
in = Simulink.SimulationInput(mdl); % Create simulation input object
in = in.setModelParameter("StopTime",num2str(SimTimePos));
in = in.setVariable('StepTime', 0, 'Workspace',mdl);

% Starting guess: From Sys ID toolbox
J = 2.10; % scaled by 100
b = 0.933; %scaled by 10 for unity solving 


pid0 = [J, b];

options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt',...
   'Display','iter-detailed', 'StepTolerance',0.1,'OptimalityTolerance',0.001);
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
          plot(stepsq, 'r', 'DisplayName', 'Reference Step')
          legend(AutoUpdate="off")
          title("Attempts at solving")
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
          in = in.setVariable('qStepSize', Tests(1, 1), 'Workspace', mdl);
          in = in.setVariable('wStart', Tests(2,1),'Workspace', mdl);
          in = in.setVariable('wStepSize', Tests(3,1), 'Workspace', mdl);
          in = in.setVariable('Kp', Tests(4,1),'Workspace', mdl);
          in = in.setVariable('Kd', Tests(5,1),'Workspace', mdl);
          out = sim(in);

          simdata =[simdata; out.yout{1}.Values.Data; out.yout{1}.Values.Data] ;
          in = in.setVariable('qStepSize', Tests(1, 2), 'Workspace', mdl);
          in = in.setVariable('wStart', Tests(2,2),'Workspace', mdl);
          in = in.setVariable('wStepSize', Tests(3,2), 'Workspace', mdl);
          in = in.setVariable('Kp', Tests(4,2),'Workspace', mdl);
          in = in.setVariable('Kd', Tests(5,2),'Workspace', mdl);
          out = sim(in);
          simdata =[simdata; out.yout{1}.Values.Data; out.yout{1}.Values.Data] ;
          

      F = simdata - stepsq;
      plot(simdata, ':')
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

mdl = 'LSQ/AK109_simpFric';
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
%   data: {"layout":"onright","rightPanelPercent":41.5}
%---
%[output:577f2787]
%   data: {"dataType":"text","outputData":{"text":"\n                                        First-order                     Norm of \n Iteration  Func-count      Resnorm      optimality       Lambda           step\n     0           3          17366.2        4.33e+03         0.01\n     1          12          11427.9        8.76e+03        10000       0.435403\n     2          15          2686.47        1.18e+04         1000       0.439046\n     3          18          1414.56        1.59e+04          100       0.179824\n     4          21          654.434             531           10      0.0500831\n\nOptimization stopped because the relative <a href = \"matlab: helpview('optim','norm_current_step','CSHelpWindow');\">norm of the current step<\/a>, 3.135294e-02,\nis less than <a href = \"matlab: helpview('optim','tolx','CSHelpWindow');\">options.StepTolerance<\/a> = 1.000000e-01.\n\n","truncated":false}}
%---
%[output:745d7922]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAXYAAADhCAYAAADcSp4qAAAAAXNSR0IArs4c6QAAIABJREFUeF7sXQdYFEm3PYIoKIpkQQVzzogJ85oD5uyaXbO75pyza1bWgNld85p1zdk155xFBAVFREyo4Hu3fDWvp+mZ6R4YbNmu7\/u\/f2WqK9zqPnXr1A0pnJycvkIrmgQ0CWgS0CSQbCSQQgP2ZLOW2kQ0CWgS0CTAJKABu\/YiaBLQJKBJIJlJQAP2ZLagCZ1Os2bN0LNnT6RKlQp37txB+\/bt4zXZu3dv+Pj46H7Lli0bJk2ahC1btmDDhg0JHYLZz0+dOpU9O3jwYLPbMPagJdtPkSIFUqZMCWdnZzx9+hRWVlbsf4YK1affv379iri4OJw9e9asOZcsWdLoc56envDw8GB1\/vjjD1C\/sbGx6NWrFy5cuKCoz6VLlyJ\/\/vywtrbWPUdt3bx5E506dVLUFlVO7PaEA6B583eZvonQ0FDF4\/ueD2jA\/j2lr8K+CbwqVKjARvb27VuMGDECZ86c0Y2UQL1Vq1Y60CdQnz59OlxcXBAQEPDdgJ2P+9ixYxYBdku3TwJOmzYtFi1axAD033\/\/ZYBtDNyFrw8BuymQFr9upp4ZN24catasyR4jACZAvnbtGgoVKoRPnz6xzZ\/ekTZt2hgFPg6SNjY2uH79Ov7++2\/s3r0btWvXRuPGjVGwYEF8\/vwZcgE0sdsTy0XYPv3Gx0ZAT3OmolTWSf2pa8Ce1BJXcX+lSpXChAkT2AjfvHkDesHXrFmDefPmsb9xUOdTePfuHWJiYuDk5KSbFQdWoeZPPwoBd8WKFciTJw\/OnTuHIkWK6J0O+G8EHHyj4G29fPmSAUrWrFlZf7xN4WZEfyftasCAAahbty7bhHjhf3\/06FG8VeAbFM2ZF1Pti9sRj0N84hFr4aRtE2ASgNP\/29raYseOHWyOmzZtwsaNG5l8eSFgJY2Z6tOzQo39\/PnzoPWjv1OhelR4Xd4GPUO\/UT3asEuXLq3T+sVCOXToEOzs7JhGTWBOIF6lShXQ3+3t7REWFgZ3d3fMnz8fN27cMKjBnzhxgjVtCLiF2nG5cuVMfiGJ3Z6wQyGojx07lv00cuRItj40floT2qA0YDe5TFoFtUhAqI3TMVuomSsBdnqW0znCufFNgoO3eN6vXr3S2yQ4EPv6+kq2R89Tm5kzZ9adMjiw79u3j42fa1i8L0P0ktSY+OZCtBM\/xQg3DiGwizc98eZgiFqhPvLmzYt27drBzc0N0dHROHz4MDp37oyTJ09i7ty5+PDhAwNxTmEQyFCRAnaxTDlVw+tzwKd\/E7DzzYA2AClgp827Xr16jOoRAjvVJZCnv1Mf1C7\/XdgO0SWkkTds2FBPqxefFghQicojjd4YLWOoPfHY5bYnBer0t19\/\/VW3UYlpGQJ3Pz8\/tXy2kuPQNHZVL0\/SDo6DG4ElfXhcexfSMXKoGK65ciDnJwE6BZAmPXr0aKaxc5Dl9Tn14+3tzYCcQI\/65v8maXAtnj8jbkN8YiDwlbonMCVZoSzoxGKKiuFyMUQFcW2ba+nUPwE0gWbXrl3x8OFDXLp0iYH5lStX0LJlS3Tv3p1RH3379mXaPAdyek7MsQs1dgJp8e\/if1P\/4mfEMiEQpXUiLZq0dA7m9N8E+DTurVu3MmB\/\/Pgx6NQj1mRPnTolyaFL0UCcMy9TpozB5THUntQDctrjz5ni1E39bup9SurfNWBPaomrtD8hDcOBXAxuNHQ5wG5II+fATYBFgMGBX9ymeCwc2IUgLd4sunXrxrRqDqzGqBVjSyCmU\/gYTQE7Hw9RFLwINzZ+0UigzMG9RIkSIB57165dIMDKly8fO7EsW7YMffr0AQHc2rVrsW7dOnb85zSMGLgzZsyI7du3x9O+OT\/PuXpOwXDtnKgwKSqGuO\/Lly+jefPmbIMhsCaQJJlyKobW4ujRo+wSdc+ePYiMjNTVFcqXAHzMmDGMUxf\/XbwJUL9U1xjNYag9qTWV0x49Jxe0hfXIWEA8JzV92hqwq2k1vuNYDFEJNCQhN50QYOfUBn1w5gA7ceyk8ROomAJ2Lkox4Bri2cWbmFKNnfcnvlugEwVxtcHBwawK59QJdIlm+fjxI+PTe\/TogdSpUzPQDwoKYiC6ePFi7N27l\/2dUyhfvnxh7XANnCxWCPipvfLly7PnT58+zeoYAnbaUKhIAfv69etZ39QOgTVtNMShk7UOnS4I2AnICWDv37\/PgJ02aho\/B35DAG7McoeDuakLXeHvidGeXFAXa\/a00dK6qhXcNWD\/jmCqlq6ltFvh2IQXmXKAXUzFiOcpBk25Gju1I5eKEfdpzHpHfEIIDw9nlj7Cy2NTGru4P3GbRHtwjZk480qVKrHTD2nCxKffvn0bq1atYiBKGxeBPvHNRMFwUBfSOBzYjx8\/zrh3AtiLFy\/qXYQKgV0uFUMnh\/379zPQJjqMniMLnd9++42ZuApNHEk+pNXPmjWLgT1ZjxD4\/0gau6mLWKlvlOZN1kt0J0LrrMaiAbsaVyWJx8S1TM5pC80bOQhzikOo2VN9ogqqVavGQJAKaaikxUhdnnI+3FxgF1+EUn9iqoT+RpQPaaOVK1eOJ0kpjV2KRuEPGmpfbAYqpnD483zO\/PKUQJ20bgLiGjVqMKB0dXVl9w5RUVHo378\/s0Ki\/ybAJs2QLFOoEIXCaRSlwM41eFOXp0QN0XqSGWLRokXZuAjsCdiNFUPct9o5dgJ2ev\/prkCupQs\/NZAFk3gjS+JP12B3GrCrZSW+4zjEF5HCoXAg54BIv3FtlmvyZPLGzQo5kIkpCSGgmgvshswdaUxCcOZcPn2oQnNHKbt8PlfhhkVjJd6bLFU4ry\/VvnADpHYMmV1y6xkCd7ILJ0ckojSIhsmZMye7OCVLmDRp0jCtl1+S0v\/T6YHAkf6b0zDUl1Iqhs+T2uGWMIbMHbkWS5emBPRE7ZiiHAjs6LRBpw5hmT17NrsrIKpGqO2LKRc6DZD9PlFPo0aNMvg1GGpP\/IDc9vhzpiggYftK6n6vz1oVwE4gQB8RvUT8Y5HzIX0voWn9Jq0EhHbsnGNP2hEkXm9kKhcYGIhatWrh3r17zGyOTkO0mYidkUhTr1ixIjP\/o9MKgT5p8FIgtmDBAqMaJ98IuOZPtI4hqxh+6cgpGFOzb9u2LTuBNGjQQNJRiW8UQhNCYZschPkGbaq\/xG6P+hODtZi\/F2rzGrCbWqH\/c1oQmrZxYCetjgqZqgn\/W0aTWpVkJoHkBOyk1ZISQyBXtWpVRhuRpk9WKOJCGvzMmTMZF09aPlmMcBNUcV05YMOdk8RaqtiJiX6Xay9OdUkhI1rJEN8s9hQlaxrS7GlDoI2L7Ny5ZyudUEzRG4ndniFgN3ShK0fW3\/sT\/K4au5ACyJQpk859nWvrZMJFfKOURv+9Baf1n3QSSE7Avnr1amZNQdo4gTUBGreUkQJ2Aj+yFydQL1u2LPz9\/SUFLwdshFo7BzOykOHeqsKGCTzJ9Z\/oL1NAS5Y0dE9A9JKxQhsA1RHelRCdR3bxRL+QvT6ZV5JHq6k++YaSWO1JaewasJv5jXfs2JFxi+T6TS8sv5CiD7lLly7syErxGcT\/NrM77TFNAt9dAqR9S10CGxpYSEgIC8LFL4ITAuxKtXwOtFJ26MK26A6ATtqmLljlCJ\/Hp5EL7qbalNueBuymJGnG7\/Syi4FdyLmTBk87+sqVK79bkCkzpqU9okkgngTIY1NuIeqErFKE8VOSEthpnEKPU6lxc36dbNmVRns0JAcOxnI5flPylNOeBuympGjG7wkFdh5W1IyutUc0CSSJBJ49e6a4H6JOyP6e6ApeKKSvVLFU2F5+kWps8BTjh0eBVDxJAw9wMJZrgmiqX1PtSclPSMWI20+scZkat7m\/f1eOnQ9aCtjlUjEE6kThFC9e3FwZaM9pErC4BMgrNDGLFC+emO1rbcmXAK0Fmfyas3nL70VZTVUCu5h6MXZ5SoBO3ohkLaAmwcpdhmLFijHPQ238ciWWuPWSSv4U4MtShSgbqeiMcvoztUEIHZqoPXF9U7+LxyCuT78Lx5\/Q8Zjqz1LjJy9UNeGPKoGdFkeuuSMH9kaNGqlKsHI+KqpDJw467pJ52Y9YtPF\/31XT5P995a9W\/FEtsMt1UFKrYL\/v66b1rklAk0BSSECt+KMKYE\/IAqhVsAmZk\/asJgFNAj+GBNSKPxqw\/xjvjzZKTQKaBFQoAQ3YLbQoahWshaarNatJQJOAiiSgVvzRNHYVvSTaUDQJaBL4sSSgAbuF1kutgrXQdLVmk0ACmsNbEgj5B+hCjvmiWvFH09h\/gBdMG2LSSUBzeEs6Wau9J8pIZcq\/RAN2C62iWgVroelqzVpYAj+6w5uFxfOfaZ47rpnyj1Er\/mga+3\/mVdUmKkcCav1Q5Yxdq5N4EpD7Hsitl3gjk9eSBuzy5KTV+o9IQK0fqlzxi1MS8ud47lZj7fBnqQ5PGi6336SsJ05ByPPx0hjIY51C\/g4ePDhBQ5L7Hsitl6DBmPGwBuxmCE17JPlKQK0fqlyJS+UuICCk3Kqm0gpSPQpmRVnL1FqkcjVQBrZNmzZh586dLB\/v\/fv3NWB3cnL6qtZFlDOuH\/1DlDNHrU7SSeBHf5+kgF0cRE+o1Usl\/uYaME86TtLnf6NIrDxE74sXL9gmINSgeTJzHsjvzZs3yJo1Kws9zE8Bwv6FSc6lxiVOGC6OBCt8M\/g4eF9BQUHs8tPe3l6vf2qjUqVK7G80NuEYeHty3wO59ZLuDf7Wk6axJ7XEtf5ULQG1fqhyhWZKY6cohMKkNUIt3dB\/C6OtkkbfpEkTHUhzkKTTABXSmI8cOcKSQxOokmUJ0SI8qB+lBRRq1Zw62bx5s8Fxiecu3HCEFBPFrudtL1y4UDcWSq8pHCdlbGvVqhXoWUNavtz3QG49ueuXWPU0YE8sSWrtJAsJiD9Ur7g4Vc\/riZWV3vikOHZD2jJ\/kGvtZAHCqRghePJ6pLVTog9htjMx3011SWtfsGABA3aet5hvGvR3qWxoUuPm4xJr7Xw84juBc+fO6YCdNgqurfP6XA40R9LYOTUlRUHJBWy59ZL6JdKAPaklrvWnagmIP9RBHz5g8IcPqh3zVDs7TLOz041PrLGL+XVjuQ2EAGfoElJMhRji5cUJ6eUAuzAdphKB87FyLZ04dvEJQNieUHt\/9OgRo5LEdwtyAVtuPSXzSYy6GrAnhhS1NpKNBKQ09iyxsaqdX7C1NYRauxQVI8xtwAGXUyRC4O\/WrZsO4OjvJAvKTkaFa9\/030KNnUCSUzNSGrNYY5eiYqhNruFLjYvAlxfhuLgmT\/OjfKtCWoWDfHR0NLsHEG5IlNZOasxCSxq5gC23XlK\/QBqwJ7XEtf5ULQG1fqhyhSYF7BzMQ0JCGMgZuqQUa66GLk+FwE7jMnR5KkXFUP+Gci3IuTyl\/sQ0kdjckbh2uqg1dnlKwE7UDF2sapenct+uJKz3o3+ISSgqrSsZEtDeJxlC+sGrGLOs4VOT+x7IrZfUItM09qSWuNafqiWg1g9V1UL7wQanAfsPsGDah\/gDLNIPNETtffqBFsuCQ5X7HsitZ8GhSjataexJLXGtP1VLgH+o5M0oJ2yrqiejDc5sCVCUT+LptSBgZoswYQ+qdcdM2Ky0p7+XBLSwvd9L8urrlyx0aIM3VtSKP5rGrr73SRvRd5YAgbuWbOM7L4IKuqcTm6lTmwbsFlootQrWQtPVmtUkoElARRJQK\/5oGruKXhJtKJoENAn8WBLQgN1C66VWwVpoulqzBiRAMV38Pn\/GSRsbPU\/M5Cqw\/9p81bqOasUfTWNX6xujjUuRBAjUt0dHQxw7RVEjP1Blmu\/8d+\/YiItlyPADjTx5DVUDdoXrKXQ7pkcNZYBRq2AVTlernkAJjMwQiyquKRBsZYW2dxLY2A\/weHunr2jn\/BXhmWwx8NDH\/8QpRY3Lolb8UaXGLoyrTIF5hIGGNmzYoLe+ahWsGl\/C5DymiFevMHNqXhxolBHPfC8ne6Db\/uYN0vnYY8ifRZG96DmsTZ06OS+vauemVvxRJbCLQ36K\/y1cZbUKVrVvYjIcGPHNO1PYo2\/+Cph18xgmf4pM9kB36fVrNseWMTHsXqFX2rTJcGXVPyW14o8qgV1KYxdHlONLrlbBqv+VTD4jJL45el8rhJ4chP2DcrH45MlZg6WNrPr8rHi0ID+uNByPXM9r49SyF8lnQX+gmagVf1QJ7HxdeXhOqbCaGrD\/QG+\/hYdKWuvDy7641f0Bjh4KwdjmWbB1b7SFe\/1+zROw55jbidFNUc+7odq7Dlg75\/r3G9B\/uGcN2BUsvjgZgFSMaTGwk+svuQBr5b8nAcpyROBO1iFFt+bFm4vv8HBccLIVBJ1QHNoswrF3L+BWfBmquKTAkj5Pku181Tox7qEsJ6ZMUs9BlRq7VFhNYRYYKY6d\/rZkyRIsXbo0qWWo9fedJdDa1hl7++1B1J\/d4NBmIboGtsPEFze\/86gs1z1tYvQ\/\/\/Tp2f+T2SNtauL8p5YbgdYySaBTp07o3LkzE4apYGFJLbFkA+w8I7qp2A5JLWCtP8tLoKh\/OTwtOAsvJ\/mC5yh1dnKyfMffqYdahVPjVWAenCl1FfZ1RzNw37t3kgbsSbwepLHXrl2bgbsG7DKEL0XFENWyadMmzJs3T68FtXJcMqapVUkkCVQa7omwzLa41f0hMnV2R+Yu7gz0kmvJ51sS6ed\/RPNBj3ElZ3vszVYL9vPq4n6YenOzJte1UCv+qFJjp5dAmP+Q\/q05KCXXTyPh8yIqwre6K9Z9SIFAzwFwrJILKX5rlWw12LYZC2J3x+X4eU8l\/PHsM7tX0GzZE\/4emdOCBuzmSE3GM2oVrIyha1USSQKFWv4BX48oFLwwGRFFO2FnTAVE\/lE\/2QI7OSe1G3oEkQH+TIJk057cTTwT6VVJ9GbUij+q1djlroBaBSt3\/Fq9hEvAp98S3LkbjFp3p2FmxfRIM\/UW\/NOlY447ybEQ9VQqb1ocGxSE8zaF2IWx58kWuHr0QXKcrqrnpFb80YBd1a+NNjhTEiCbbtJY8\/6cAy92RbLqpc4UZuaO\/N+m2vjRfi\/WaRyqV1rNgJ02snrZ\/0HGcxNxff9Ws6ZCMuRFs6xRJkIN2JXJS3ZttQpW9gS0igmSAIHS2QIp8aiIE8qse5kowE5ttoiJYfSGGkvt5vNxMmVKRP3VjQ0v34Ls6Nn2mtlhBWi+gek\/4eCLr6qdsxrXgcakVvzRNHa1vjHauGRJIGs6d7ztvRMdb\/XHtC3H8HfrAujqvQIOq7rgwdPLstoQVyKTSYqe2KNfThyZGGpWG5Z8iAKeUWwYHjaBLo+9YmOZXbs5hSJjxnX1wK4yTsyyKObZJ3Oa+U8+owG7hZZdrYK10HS1ZkUSsMvrjbSNNuHd5ib4cPsbNVF52xOzA2NxamdZE09smpxbdWaTND6Pc0VRf1ckho0LRhmPVHjXdjO+XDuDm7vHm\/V+8IBitJmNf22drOPsmCUgIw+pFX80jT2xV1prL0klUKCoHewX5cLlBrd1miZREzHPPpsVVoADO12+UuIOtXl0pvZIxcwbay4Jw9jAMDTNbYfo+gF4nTocpycMUSx7mu\/8t29ZgpLBHz7ghI2NRscokKIG7AqEpaSqWgWrZA5aXfMlQF6X03p5od\/RN9h49wNryLHndmQJ+wdXNy1Q3HBOd2s4by\/ANoXbqx\/Ad3YRszYIxR3LfICoJ4fWC3Fz9wR8fnIBmdNZI6ZCeuz765FZmZQo7ky+BTlw0iYl4+mpaCGAZS6GxrHLF5TSmhqwK5VY8qqfq0BtRNYfi1zL\/XDq2Semwd5puNFsaoL49aNXS7FIkcVrOeg8WtUiNQLiO6Mvs7g4BOxUeFpAc04XfL60kfl9+YJn47LpnX7UMm+1jkOt+KNRMWp9Y7RxyZJAmvJdkLpwPZ2zDnHO5R9G6aI9ympEUEl4EdmgRjoGdGoKT8CpJ7vuD3Dk4recpySD9DUaITKgvuKLT2FkTB5QLDnH2VH6PpiqrwG7KQmZ+btaBWvmdLTHFEpACMRpM3uxpzPUjDE7Xgx5dVIhCxM1BhTLkbkoXP6Og1WD2+yEQmVo+xYI9OyP6OV+ZgE78ex9PHOiefhjLVKkwvdPrfijaewKF1Krri4JbPa2YnFifK7lRJmZ87GtbDFMGpUF2+o4mqVpD2ibAfuaV8PVOht1FIeaNFhOPX1Y1RilZ8zHweb+oAvVn4dmZPSRUm\/bCfmssGF6YwTv\/hmfgy6gSse1uLA7CiFLwtS10CodjQbsFloYtQrWQtPVmhVJIH\/tkUjh5YUbC7vofmnXqQ8u21Q2K14M2YhX6rkOj6LDUGTLCHy56KMqL1ZOnbQdOwnZm7ZkGxkVsW273BeFNjJuv85OKptz4\/rlD6q6MJY7l+9RT634o2ns3+Nt0PpMNAmQhQgV7oX50\/rtKPr4IYZ1aavYVJGbOlYZdpRZnVAMFrdiy+DYYK5iTTjRJihqqExHVzi3csPlhrfxNDoWhfoOQv28XlgYVAr559RmWZWUFG7Dzr1stYBiSqSneZ4qk5aC2mrdMRVMQauaAAnUmJ8VjyNc4dNhD9Z4OTOgy9mzBkIj+iq27hDasHNKw1xNWDil9MXTMrv6xPDodGzWEw1bHcSyBrdZF6S1l3x8BgeqrYfr\/p64de6sbGnSfHemsEeV4UeZlY2Nd3EQ1dNlRvUEmTy61nFkY0iusXqEAlYr\/mgau+zPQKuoRgmcruWAGL\/yWNR2JaMl3j19AhsvH6Ztc29UueMms8HwNWPxekOADpRIg208s1CCqQkKTEYcOOVjTUgh6ul14TKo1KMUDjTzR\/jpk6w5clqqXuOSIuciAvbq87Pi0MuvCF7hBqsMHshbthNqzKqpqB2p+dB81WRNlBCZG3tWA3YLSVatgrXQdLVmRRIo0m0LPjzZi7u7v1EyVFpV9cO+krMVUxME7JFb+yBkaxFG7aQuXBfp6o6Ga5bmLIaKGgpZ7fRfFQRff089YF\/drTeiZkxRpGlz+\/fCg\/LpLkvVaOKpBrkbGoNa8UfT2NX81mhjMykBl2HnEL1zLGKu7tSraw6FIrTppsa45t9lejVM\/vTa5FikKmQflQUxoZ8SzcqEwiW4Rcfh6KDHuu7IOibkkx\/eXlqCyA0Bsscp5dgkloHsxiQq0ikiuQcV04A9IW+IkWfVKlgLTVdrViABzol3fnUJby68Y6Z\/VDpfeolbsa1wd\/Q5RTwvJYnufuKFXpTEhERO5HFdiIKhQq77wpg25iym28BlqGA3GpvGBf\/\/CaVPO1y3a4rgR\/8oAnZySKJk2Bvcsuoun+kyuu+dExhz\/k9zhsdCCNNakLlkco+LTwJSK\/5oGrtZr6\/2kBokQMB+sqsHNvhPwopDd3Bt1jQ2rGNNFmNatzm4tO6lIk2ZaIgTPofwcpKvbnqJqcEmBs9OcXBy\/5Qf9p9v6jayzbXDsKJ6H3TuGKgodC9Fczw44i9EHrqHtzvHsjkTsDuV\/4i3O8Yq2hS5wGiO\/Qbfxta90f+JKJEasFsICdQqWAtNV2tWIAErBw849dyOdNt64tGN\/7cG4aFsXbf\/rwPPrT2yZbYgZXqMHHRQLw4LufC\/7zYZoWMHmWXVQhsDFUqModR5SGrgdJm7dddBnEhpowN2qmfOBkQnlFeBefROEUTPmGu7z09QFCee2qHYNcm9qBV\/NI09ub95yXh+nAN\/FeAPl08psbDyeTTa7c5MAIl3Lutsq+gykS4mh3f9E7dPLtUF2CKqwrWuIx51+02xJDN1dodPbQd0rno+0cLhkkacccNrbJvxBONKbcGoMw3ZuDr2m4xzzxxwbW0P2eOUopk4OJsTUIye3fbmDer\/XzgG2sx4MhDZg\/rBKmrAbqEFU6tgLTRdrVmBBMg+nHhrb7e1uNlpPMbZzsHI0w3xIncKFF7sh4eb1ioyU6T0cPfCYvVM\/RISOZH4Zq\/YODh1ucM0ajKbTMhFKj+hFL9SHft2RYIomPX3fsf6e9NRrctE3M7njeB+bWS\/I1InFG4JVG10YcWgTBsZbYJ0j0Cb5Ad\/F8ycmjdZmz2qFX80jV32Z6BVVJsEmrlbY4a\/C6b03o98Ozug17BvYWypEFecMk0EIgKHyx62b+9f8Tb\/Dj3TRqlk2XIbJHB7Ym3NTg3VSneAU95teNw9s9lAx4G9hL8HLv7WHAudNrON7Marf0FOQYu+3mYnFrmleXsn3Gnghht90upOKPwUZA6w0wkgxDETS9oxN\/Q+G0Zyj+2uAbvct01hPbUKVuE0tOpmSIBrl2lWVsSTkPd6LRA1cfr2e0Xp4hr37IVDqK4LAUwN8j5Sh9dVdBFLzxKwExWx3i0rCrX8A47bR+NI6FUzZvrtEdpkDqbPgN4HbmKMYwBK55msa4uoJ7JymTGkL55YWcnqg4cTWONmz1LsUYgCihjZdmhGnPNKo9h2n9rb4tsY86r\/ipoza7BIkWoKoCZLKAorqRV\/NI1d4UJq1dUjAWEsduKbqXDOWellojCcQJxXWl1IXA7sSuOwkKljwTXlELR\/KrOzn3FqfYI1WE495ZkbiVnDfVGs2h7dhub1y2S8d6mqKHQvB\/ZHRZxYrtgsgd8iOorjx8hdcfIdoJSCdElMdxO2hesiR\/1OqgqiJncucutpwC5XUv9XL1u2bJg+fTo8PT3ZX44dO4bBgwfHa0WtglU4Xa26GRIg8KbYMJvadkSt4bE4\/HQd45up5Bnry+Kyv61lz6gKU4UD++Q8zvi1gRtmXniLZrntWMq9P26FM82bB8oy1Rb9zkH4wbaloMtdinU+MkMsOhwpbXZoAU6TOGyqgHaOa\/Q2sko1GuG6z1DUPrEFq45NkjNEjCnRBnNKNdM7odCDZFJZ\/NAfiiyKeJLtp4Fh7GRDGyKFJ\/CuPjhZhwFWK\/6oVmNfsWIF0qVLhwEDBsDNzQ0TJkzA9u3bMW\/ePL2XVq2ClfU3yIfTAAAgAElEQVRlaZUSJIG+xdOiw095MKv+NEx3+gP+7Y7h5LmXrM0efT1xoYULSnbpjHlX+5jsh4Dp6+w1+HiwN7ans0aZdd\/aoUIaLGmhSvhifunKqQj6N1ETnS81wpPpl8yyEa\/UpAau554Av7opdeF6+RjphHA4\/UQMvP6bLLNKmm+xTuMQ4+KFPb931JMPAXuGQg8VWQJRrtg0yxrh8cjdung45sjN5EKprIJa8UeVwF6qVCmMGjUKK1euxIYNG4wupVoFq7L3L1kOJ3\/XqUiRwkkvFjufKNdghc5GxoRAwPu2907cu7Eb748H6lWlULnhRf3xoM9S2XIkvptOFEJbbkPhD+Q2Sm3W\/nUAO6F0uvsrnoS+010YS0WmNNYu1SeX\/4phX5BlVySa7Yxk+WKJkpn6KgbnBnkp8pKluRI1NitPOZ0XK21kVJRsiHJloZZ6asUfVQJ779694e\/vjxEjRuDMmTMasKvlLVbZOIqNmYWH9+yQ+Z9VGF96C7odLoHwD99c7bkt+4nMjrIuEwnYt1l9xPrfuqLnvG80Bwe6wqkC4X55G7Nvl1toM3iUfRXe7N2s2ygSGuuczAmLtXDB2XpvcHhhHxwOWacDdr+83rjTaBO6R7pj\/IJvKQJNbWTbo6N1nLiwrtL7CXqWP0NJSnhsfOLZrR08dP82NaYf8XcN2BWsGgF7pUqVEBISAl\/fb+7dGseuQID\/kapkJ57JrwkqnRiIDoWHYcuxizqg4xx3tzl\/YvCWOjrANyQa8jA9fD8SD+pmZrw6WYdkTmeNUy1ccGFJEKyiPily1y\/XtzBu2y3V82IlYN80OTfj6x8KYr3IXa5q\/YYhzKYsYgMnsY2MmzryTehwk2No\/8QZF44MxeGn3y5rDRVjJxTSvCku+5WF35yf5JTe1dLg7ISceqac\/OKZLlCTawhfDdjlvB3\/V4eAvVWrVjowb9asGXr27IlNmzYZ5Njp94sXLyroRav6o0tgQ11HuJf2Q6+I\/Kj1aQe8MqXRATu\/aBwSao1dB6aaBLpqOTPhUrOtklYlpI0evVpKETUhZVnyU76auNJwvNlhgDvNmIB9n+ogd\/ZgFk7AzS6L3oZFfT7zv4vax0qaXFrixJ23F4h3kUtUDAHy6HR9keJKddl3AdzZSUh9EXVEdIw5XqwmJ6CCCh4eHqD\/BQQEoFGjRnj27JkKRvVtCD8MFUOXqVTat2+vJzy+Y9IflyxZgqVL5R+XVbMK2kAUS4BAvOCg1bi\/d79eLHbeULYCJRFdPwD28+ricbTpxMyVPAvjevtvFixxUfofKFEKjlVyIfz3jrLjxUiFDeaJqKX6kCMAAslny+7i4KeTyHb8d8yf5KNn8thh7RbcCg3B6f69TDZHoDvH+Ssu9suFmoLTAwF7M\/eUaNw\/syIzxZ1W9mg\/5LDeCSUh4QlMTkAFFTp16oTOnTuzkWjALmNBSENv164dxo0bp+PYCdjDwsLimTxyYCerGdLY1bRrypiqViUBEiDrjZirOzDkU228+BAcz\/pFSUx2As2GjTLipE1KdpEoLEJ7ebnDLTdpAG6EVNAzJUyoBkux3VN72KDwzEboWrM+inSfpQfsJI+0qW\/h6cz4ZsHicXNO\/GO3nHoWQFSPW\/S4ZisQb5MzNH9DsqZEKO\/LXEDYnFkJzh4lV\/ZJVY+09dq1azNw14BdhtS5Dfv9+\/cZkBPQd+nSBYGBgfGsZNTKccmYplYlgRI46JABS689QMWZsTi0a5rOhp03SwGzfG8NQpUN1Uy62tNl58a3sZhxPwazBOnr+AVqmqm3ZFMKPELi7bk99C4OSYNdmQcY8mdRRbQOzYefUB6lGoiqC36Dq10WnTMWn2\/hwTuR87MDql+yw6jTDY3eK7S2dcbefnv0NGzeDg9dIPe0I0yxJ747II\/btxU\/mR0GOIGviMUfVyv+qJKKodUQOyitWbMmHr9O9dQqWIu\/UVoHeNrMCfvq\/YbeQ5egqMd7bF9ZQafBkou88+pcSLOxAloEjzEJ7IWbdEewey2kXdWQudYLi9JAYGIbdmFbe+0\/otv0hYhYPFyRBsuB\/WmGAix2+sONa+O9AcLYNKZejxo+JXChxgJJ6omeJdPM9Ckbybrkpfmm+H1ovBMKtaNkTKbGrMbf1Yo\/qgV2uYuoVsHKHb9WzzwJDOqZD8scVkmmxaMWyaLlY+\/TBn8X95rXrxOeZfWVNM0jjfT9iAv4cqoVXh++Z3LAxswFKRk1xXLhiS1MNiaosNzRCfuv3NPlOr20vybWbg3CtIBbrJaSbE+UVOTLQC\/kGvRY74RC7ZAl0NY6jth476NsYHdos4jFnOemjnzYSsakRBZqqatW\/NGAXS1viDYORRJo3z0ndjquRc6N4zEoV6Ce6R9vaGHdBjj2x1Ks8XJG5czNjVrGXEj\/CW7NszC+Wayxn+lWGHWclsJhVRc8eHrZ5Di5hU3Un93gHnmZBdiisvHuB7O8WHmH5FBke\/QN2l\/ezmz2CdhPnnuhswSi0L1h5Rpj4AFHnL2y3qjHrSnnISU29\/zuQCrgF78wlusoZlK4KqugAbuFFkSuYEnrkhv1zkJDTbJmyb2cSsyzT0nWZ1J3xPOJ5sIsNBpbkAHd3u0FmPZKWiyVGvOzIrZCM7T7vb9JoKO6S6580AXCEs6HNNisj9+i39EoWTHK65TPijPlN+r4a+LpCdS5Vr1vbzHF8VNaNvDGTd+lePmoNeaFXmWgLbZVz9dyHCIyZ0bzvW8lOXjhnCiM8HmfhvHixPA6dBH7OeiCrJMFpdi79U8RRA96jKUeqdjmSDLbcPcDtofnQlTbQLNNPJP6vVLan1z8UdpuQuv\/JzR20ij8vnxhR8XkntGFXghyziELirdd7+F+mD5fnNAXRi3Pc+ehmZXHY1arX9iwxBosxWSPjXqGDk+cWUIK7pUqNQeffktw69yZeOEEeF0C\/nvhsbKoCeKVqfinTx+vK27LnuZTFzyZblr75w0QsO\/Pvwk5Sjjiep8qyPo+c7zgZko8Rik0b8pSGbCswe14Y6SNiExFn\/tNY5e8psrQVBkQOGA\/28j+KvxQF56gX\/G0mLM1HAGhr2VfPJvqS22\/a8BuoRWRI1jyUMzlnhJhmWwVWyNYaNgWbZbHKaFOKE1ZcjypDJ\/aBouifmWXnUFPn0rKs0C3QPabKT6bTnOOPbZJxokRarCpUx7G8zmzTK4dAXusV1qUaOrJQI68WHkZlDMTljXbavDS0ljju2v5Y1q\/uThSLSurRvcMBPgUvpcKrbvz2es4f\/aUSVv281kA95rukicUAvZ6\/5sNKaBielkeo1KZmPg8uC27Z6W2iLm606TsfrQKcvDne8wp2Wvs\/Mj+aXETrFwyBZM\/RSZ7rZ00qEtVeiCm5jMEvTogS8v8Hi9fQvrUZfrZdxwV01VgVIy4yL24I\/A5VctB0oZdCOxyqYm6jbripEedeDQHp3RS7Q5VnICCv8feUzKhRcwKZuVDoE5OSs75N7Nhchf+nM5nTAK7KRt\/kt3wX1bLivNiyvKFbNnfFA1SFC0yIe9GUj6rAbuFpG1KsDzB7qCafRnYdRxZUFFcbQsN22LNcgCoWvcLFnVZCaVHfosNLJEbJhO7Knv\/wRvbsnAefjyeTTd1R3SUh38+OJ\/tifY7S0tesDJA9EjFIh2W7f5Az0KEAomRWSFdflL4gq3Lg9H7y7f7C2OFaJsn1laSGYjofRzwcwYsa+opi+bg\/QRMrobR0ZMw+9MKpLk5StJ8M2fZ+nhdaQToopISewvj04vHW7v5fBx798LgaYashF5W7MbaMlUGtM2A220zYfiSsHjOTrSZLUnbB8cqfMKTIdOS3b2PKfwxJTtL\/Z7sNXahDXLKMd5MjuYEYLLUAiR2uxykbnV\/gG4nX6Lc58+Kglcl9ngs0R7x61nsrRnYNs07UqedijXY0n098bFiegaglPiZ5wMlCoPswk+efckuWrOmc2che4kj\/vzkW95UAnW30n66tpVo\/xe\/vkP9OFuDcdHXV0qFIS2mKdJgFwTUw8iQUchX\/AuONymjJ1aaC6UGlGtvz6knl1cHcHBDAGuL5kpzpnAE9f+9BN\/da7Cj3l5Z1CUl7Nj8Uwv8tL+5pOlktiuvsOl8dLIM36sBuyW+cBkOSu6tCyDWewX7aLO0D0cuN2vs7fXYQqP5\/s3Sx31n9GW8P74Y9Q\/MjxcT\/PuPMOEjIGAfs2YE5p7wxdWpdQ02mLt2N7zIVlvS8oPAndt\/SwE7B3fuCFRw7iG8fBOD5yNqGZ0AyX+LYyxiWnpL8te0Ge1174v1YbEmuX9hRwTG\/afMwvYcuVgAMCoE6HRhzBOMSAG7n68LBvfMD\/\/2xxBxsxGmBtzCunk34HGuKHi2I2qLwDzs1EkG7GkzeyGghQsWNP1iMtsTjatGjWHY4JbVIG1jyrQy4W\/E92tBA3YLyd6UYIWWAvTip6s3Gnv2yEsdZqEhW7RZDuy0kRHnqjR4lUUHl4iNr\/6nM\/rs9Mfkm54GKQeemOKfYWOYLTvlReU5UYVDaWEbh99\/9oh30Sms065TH+xy\/9kkNcHl3yV0BiavWBdvxjzIFtE6ShJQ8DDELX\/vhTfBrrp5EFhzjp3fO5Rv7YXPnaajo9MApGzWFFkypWEXrHwz4xsAz08qtSzGbNPF9YW5TqXa4pZA5gY\/S8TXJtGbMoU\/id6hzAaTPRUjND2jKH30wu7dOylZWorQmlMyhp3T7zDzMn6ZpiT8qsz35rtX8xpQFFlauKHTrXI4MvE2Vt1eE0+Dda3jyHj2HB6H8axsD4xs2IsFzhKm0CNgbDPwKF6NyoSQqjcQ6ZCJaa+0EQgLAXLLmE\/YZneC\/Vnqspb+XiGtK27+uhsfD\/XG29OnJeXE7b6JLnsjiEtjSKh0Qln05y9od6YjBt4KQ0zoZ8kNyjtzZrxru4WdTnO\/Ta1LPpKr4AddykDqw9AJRdg\/n0ftvBmRcvkWLDssnV6QNol8C3Ig3UcrzBokfRLe0qMqumSYjNThdVk+1ORUNGC30GqaEqzwxj65hxElEZNpp1dsnI5uMmX9YKFlsWizRBVMq98A5dOcQIPHefTiphBQ9xp2gXHnfGOLXu4Hh9fuknbsRGVc+FgKQ07V1WnjBOzbyhZjc6CY581zDcSNZ7\/i3c5C8HSehfsBe3Ft1jT2ewGnsnr25ANTfUGr8Ld6KfHEwuAarNwQBXR522F6Pwy70RhDy1xFiul9WTIQqWJqvYm+CRxXCKMKfmA0i22qoigzcz5O9euF8NMnWZPNcw1AXIoAUOKOEq2P4fWe1Lq7BvF8hdSfOKUgHx\/RT+VffoDH6Y8WfS++R+Om8Od7jIn6TNYaO10kug1cBvtzD1HzzlR2sUMR\/+jy9MUu\/dCs32sBErtf8igMq9gY7U+3wqlnnzHs7DNmBZScHLP6V3bGm9a\/4\/TBj7i2todBEfKY7KYoAGNheYm+cUuTBYcyL8ShLF2Rr\/RlHPfvwvrsXXgu8\/Ccf7WPbtMY4JoCvzZwM0jrFOo7CD\/bhWMaesq2ZefRFoliIc7\/ypjfdHMmc0cqtJlRIWAffeMRjm5ez5KPCO3c6Xfi3LdMLIKYjdeZBUueJVvw7ukTHXATqLvZeSHo7RzELbZFxPH3yHoljm2UFJaBNjnhaYVr9sKLZz442oCzN22B9NMmgtLwZSxaTXc5ndjv\/fdqTwN2C0nemGAJ2B17bkOnF\/vgfHkphkZWQLq6o5F\/Tm1m6mVuoSM+HaHV6LJP3pZU0m3vyWKeUJS+FKEz8EKC71Uyf+J45dAGSto0ty7RIocqjcCd0ALY8H4b7kUeRrPd34JzUe7PdeU\/Mhf+vw9EYIRbfky0ewfrprVwdM5SEA9d4vM1Vpcch+jfa7zvMJPGzBteSf5eK+VrdBlTEB1uj2CX0id+u4d\/70Szy9e83mMQ\/WiErj3aWMmstnZQI6z6KxjpC5RC\/XYl8XnPBqw7FIbym06h2b2laHhpM7rsy43IkW3g\/DUCr9fPQw2rQIx\/NgEl3YP0qBNqnOc7PX1oBfxPncKKi5Nh4+2D9R3j8Mu6KLT+9M2LdXXNIyhY2ZWFAxiccg7iXtuz31PdOM0sib5+yYNSWVKj5qMojLMNYv9+mCM9cgSdY8975cmFLzE\/YXvISYRUe4FWX7vh6s1dqLphOx6V8kB+hw7YkeE6IkM\/MjlWdnaB\/1cX9H11G+Ub5UC+UnkxZWIwHn74ij5\/Dfo2poFz4VF0AKwyPoZTvhy4NmsqcsXegUOe+nB4ewDbz39lm8uPWDRgt9CqGROsOHM713y6TK+GyZ9emz0iAna1avx0IqlY+IzOVp9ifuDFXkT+n1mbOZPmJpRkNqiGzYzuSgrm80X23mXg89cKNHs4TWeBUnjgaTRL\/TfSHF+EoXc\/Mvt0ok8qRr\/Cz6nX4uf9rbHo+jctP0tgGNv4hmZpjhbhKVHzdQPk7Nwfc2cXZr+3uByLO9X+waUX\/295Q1zxkrgxsM7sgOpPauGZXVP2u03pLJg4py129oiDW7Fl2BHlCrfqB9HqSQR8H\/2D1jHL0btnMRxs2hw3Kw3BMKe3CD+RFbdSjsComudR7WgAe\/7e4kao5XoJG1p00IE7nVDSdR2OOcdzYq29Gwq8qs+CklHZ9XIQFrbZibtdC6BVVT+MTjsBdmVc8OB8JHa96gT3iIn4fbgP2kwpzCJexr5egva\/FML1wy9w6HNXpH88FsNmlMXU8flZe7YdamCPW2X0XVYQnrFhKP4uPT4NKoDsQWfhYv8WrT\/swJg0U7B+QwhuFO+L569S45ciLzB2VySKFfZG35LbWDv7Fj5k8yQ59bYPhO3sxXjZciOWvCiNOXYDsGh\/BLo3f4sHjoUZqHPqy5z383s+owG7haSvBNjJWSKwhQuOXHyXLG3ZOQBTjJgblz+AX\/g93\/PcKOdrbGlIU0zvk5ZVebkzUhUb2rpKk1A22x5c9MmHtgsuwyqDB+yv7sDGOo64UNgbXp5pmXkfbexk1vdpViguUWCqDa3h1+UQPOPC2XzoREObPZ3i\/Eq6IiajDQOZp2N+Ra1CVTFmmhcGTN+H4we\/hSy4EvQCfTxz4ajNazh27s\/svnfUqs3a+2OeH4b2+4Jl+S7jZoZsQMxkBP9vXPcxt3vo+vPL2BcRqWJx+8lcOLfexCJF0glAOB5qc\/5EH6weNE8XzIwD+9JHlZHt9gr0iliFzWGOaOQeyeZQeURZZvVCwL39dSrsdLPHmC8p2Zg9v8Qi0tEWjlEh35Y5zhmvitdH+hqNUKuSDbN\/JxPHbm4tUdDJDyNPdWb1P0R\/RtWavyC40mo8mfiFPer89SX8fF2Zhk2\/b52TC6cvZsNi+17oHb4HXo+L6D0\/ueBwrL+zHHdsX+CfTKlw9WM41qZKhcvP0rDn7dLZwL2MHzOz1DT2xAXIZM2x25cuDdsq87AoqD0a\/3WDSY6sJLxiY3FkYqhiSRJwco2V2qGiJmcnHjd8bPQsTF6xHk1z2aKfjz0o+49USFU5AqDL2DcX3qnKmmHzoDxYkmmSXr5TolJoIyPeWFhII4\/eOdZonBIeLEwcU4b4a85dU5vkGv\/k1YF4px\/ikqkQeJLyQIHJHrx2R\/SkNQZDBdesOQye+dNh2cyh8ZbhcdMdGHBuNjY9Psh+E55QDjTzZ7QJjz\/DnbII2AmkKdzuSRsbPBlUIh6\/zjvijlvL02ZlGwJdqLZo4I1z2wrrjZcChZ3zShPPg5Y2Hw7GPs3H4qFLMXiuncKav\/HqX8nXiowYTtjYJDuvb01jl4MiZtQxJlipTDZETZR6cNosW3ZuS0zWBOmK2yNzF3dZQZLMmJZZj3BnkcIeZ9B71WEdKJAte3C\/Nma1qcaHyB57r+c8FH\/ghVKP3ugu80hjFcdSp8vEme0749H8QGbCSK72FOlRGPI2uIs7Zl54yy7XyUqGCg+sJZw\/pXl7FB0Wz7GIrGioEJ1Qc+AyXP50zWSwMIr\/TpfbUrbsRN+QFi3MkkSKhE+BSmi5crpeaATupMRt2WmTcsmfHaV+9kG1I52xZ3wOdhlKYX55EUefNDRnvkmIx\/jT+u3MSYrk3bB0aoREFsLm82eMvipkCfTCvxfSbeuJk7e\/hVVODkUDdgutojHBisOY8nCkuHbfLGAnWsKntgPS1rn2TSuysrLQrMxrltttP\/O9rBsbc1Jq1hORAfUTzI+rxaKItHDvaoPh6lYHDXpV1LPSEGuw\/ptzIzJtbWTJNxJBjfOgeMwoJlwOdARO1VfnYjz8qdDPqLxvLcqm6YEq88rHW4QzfmkQlyVNvFMBVSSt3fnJYxYlMiz1MZPAThmMno3LJqkYiIH9725ZsTTfDLx+kAsD7ruz+RoKQUwZmr462ODW2m\/zJEsWst7h4RTob1Tnc+1muNerEDPn3DpwHsqOaxBvvu3zVMPOxpMMOmWR7PpO88Zqj1Qm495QjtX7RzOh5pIwjA1MPrbsGrCbh1Umn1IK7HRct1t4Hy7W6Uy2LVWB3KPJweOUsxcmvrhpVhuWeog2nhp1HXHA\/6YO2EkbpWiCvsEpFG9E\/ITysrEVnmXzRe7+x01+wJaam7BdAna6GDzrXZLlOxUWsZs9ndBiru4wGGednqX2Pl2YjDd7NzMbbirr702PN5VKwz0RltlWMrgXVSZ77Z9WPYAxj07eKLdllwqyJbTFp\/oc2Cnf6YPh1Y3y0XJi2rRekB0hpcvB+m4TuIw4Fk+jF47x7YpQ5rglPgnxOqaCifF65Bg4O3tKuIdYJ8UrkmR9aMBuIVEbE6xzl4nwzeDBPPE4Jyml1codGo8U2Td\/BVDKr\/2nl+NxtHq0D+6Q8\/OeSrpgTMT5UhCmvpc\/KLZl5yeUAi3CsbDLKpZpngfJkiszS9QL21QOBS7OguO20SyGurFiiD4RbxQc2I21RREPw4rWN5h1iJSG1vcjZW2i3P478PVQrDlwUpdhifqnVHzTD93T\/Y02p1\/PbMDdzXMY3SO+aBQmGCFgb+njhLn\/HMbezRfjhe\/lCUpmH45kicDJ0cpQ2sBahVPjVWAe2HV\/wAwOxEVOHHshsJNHLq2XIUcmS7wrlm5TA3YLSdiYYIlvLOOZSg\/YOfjlGVvUYPQ9Q0MtuKYcftl1FX3+esMuqUYfKqk4xZm4bdKKY559TjBNQu1KOdrw+CFyvRzF4+OXXhQlcqqdHV4tyWNQY7XQEus1S8D0d+sCzNyv\/303hN7eZjS3J62T16ZauJwtO4ZfLqMDOqIniI7hIXn7rLDCls\/PjE6Ba9mVN1XQA2L+UI7MRVkauK+3+iNiyzGjbfH7n3qZnfHv+696daU4drrEbvC+Prx3to4XzkB4ycvfgTH9UjDrFR4wTBjdkmRCDmvkuGasEJW5bPx1g45UdEL5+vYd5u5\/Z\/Jb4psAWQLVujuNKVp0CiB6lDItiS+9k+JdSow+NGBPDClKtGFMsFIJADiwVxtdWLEGW8mzMK63X8o2ilm3jjHrGqn0Z3KnSqCeqYs7q36r+0O5jxmsJw4nQBW57b7r\/p64de6soj74s2SS12vfXMR19cDxnl6JcmFsrsMTH1ONIp\/g4pwHcf3WSAI7BzuiT\/waZ4ObwxBmPkiXkpx3Jq66esko3JuWFaWjDyHfxMeYd6WPQcsOAro+7TJhxv2YeOFpSbD0+9GrpWR5NnNTTO4FTdo+FQoVIAb283OKo9PbafEui6UWUyp5NIUB6F1krm5DIOueqIfXMTPlL5K5U4Va9v6xV\/XCGfPfuOY\/uo6jbPPh+h3G4FqWpeh1NAob735k4M6siC6+k9woFb2s36myBuwWErwhwRLXOt8xBU6ce6mnmZCWMWNkFlkfnyHtlTQdJdHvLDT1eM0SsPvbW2Pqz9+8MKn0\/V+PUTJ53Db7oaJogvQsOfd03BiK\/sdzoeSD05iQ3xo\/hWdCXJRxzdbQfLljF1l4ELDLyacpbqu0ayGsq9IP1umHYHyuLlg8aEK87gjUyd6aLFuor5y5bPFubmQ8b056kJ9oUpzrhxf7jxtdKgLuwR8+SJqOkpz750yNF4sfyvYZoDyrd+4GI8PRCcw0NUs6awbsdC9C\/88TYD9fWgI1nk9BbDorlu9UqhAvT\/P1exgJomMMmbfSCeVeh5NwtDmGn\/dklwwmJgT2h5d9UX9XJIaNC47XLQ8mJueEQg\/TSUEc3kLKkimpvpfE6EcD9sSQokQbhgTbvntO7HRcizphq7Fy6Vy9J8m64\/OYIFz8J0r2qAiIjh4K0R1hueZP1hkXKh+S3Y4lK3Lq6frCLrrLLjrqkjaoFNi5s1OToXcx8NBHlsSB4rybsgk3Nj\/aeKgk5HRCwDR8Wmu0v9VDUpMU90824NkLGI7bXr3sNFysVFlW3JY8ubMgoslmyX5JYfD7\/AUXNobIPsWRLTtdwkf91U1v2EJg5yeUlo538cQvv6SHpvDC2CvoDeYfqauLYMmzQPEONtUZiRV9NzOHLVORFkl5IWDvefQNho0PjneByk+wpmLx8L6TY1x2DdgthGjGBGso0h1ZQRTZMhIHb31LAiyn8EvXlMUv6PhEyoQzc2pes6gJYaYj+m\/SLBPqsr\/d4Su29Z2NpWP66k2JLt+yhP2Dq5sWyJmqrg5pqMIMTKbibptqPLGcnVbtqYh+x6azWOxrzvcz6ARE45nfPQ\/8GmVj2qyY4iALmNFlbsEu9D6yHbhjavi6zY0cwHrOWxOvftUav+BOjQoI\/72jrDsTAjp6f8ROblIcOykjtbcNZN6\/Qpt08SA4z09Jvotu2KFLnsHrTSs4Cz+9vo8+j2ab5MWNZWSijez8xlB2cqUQ0XIK5eINHLBf1iYqpz011NGAPQGrsGLFCvZ0+\/bt47VikIqJi8Ol\/HbA5jrsoyavPCqkwVI4UqXAzjMxCTVWQw4c5kyVaA9zqAne1\/YVFTAx32vJkwhp8inTRCAicLiiofGAYndmHdsAACAASURBVFyjpPl2u1PBLBpL3DHNl4KKKfXcpZOSW\/sW8PSbhlpn\/8LRqRMkgZ0sRShIV1xtezwuYs02X3IkIqcfHnKXxjS6izv2dJbnaMaB7v3gfJLRG0leTuU\/yk55R5u5OKMXce\/lnkbqHJRYmIB5NVH97hgMe\/QFZ87\/LWmKyeXLqaU8m5tIOgIVn7UeT15klwWuNJaCLf+QzI1K3HhEZAzGj8tmMssSH1vxWg6wGeNtliKk6MVNwsoasJsp7N69e6NVq1a4c+eObGCnoymC3zJOD8876+Jz0xDGLRiNDhcXyk5MzIctzEzETf5IE5abud7M6ct+bO++nzDK4QUD9qrZPXGhwlSdRQQd7cP2hKFEfJrUYPtEPaVptInZdnPzNKI1KEyDqfRwUo0KwzHQ7+aGZCCrjwqly4DMBGvbt9MDaWG\/xLPTZr7lSDZE1h9r0MmG2qO4KXLmJL7w5P1xyw7bhfdlWZvw58icNMZtpx69RX2c7Oqh49gpecbWuSVR7fxQeLk+xMW+zY2+Ey3dUmH+7W+xgcQOdOREtWJgMUwv\/Ug2GHee6wVnx9Qs3g555goLOTqFOGYymBJPPFDS7ukUWD99er2x\/cjhBjRglw1R\/18xW7ZsmDRpElxcXBASEiIb2CnmdJcpLTDgYDtU3d8cZ7OWAsXBpmQCFH70+p56cAp6rChejNiLlUZJQJctnbvRmOBKpk0fuine01h7RDFNcFiGog0yYHJEMxxoVp9pqUPmVUOmoQcUxYvhNuwR9W7ojuxSFhdy50dUFlkAJeRUwk9cNWYM0kuMYWwM\/C6EqImgp9+CeQmL0jXkF57CuDJ0h0GaNTknkfu93Nj3UvcW9J717pNVB+xcA9\/4Z1tM+P3\/N2upOVOohXIZbeHX86AO2MW0DlEoFR5EIeOtOFlLN3ywGx5ntceUi+\/i5XAVn+hMNUjzzevXGevdsurCMogjsJpqQ22\/a8BuxopMnTqVPeXu\/s0kUAkVs+JgLwbsZHP8avx69jy36WWXjBFPFIUVyPhrX1RPsxarJj\/XzYQ0kHe7Cplly06c6bONVoh61g2xUc9g93Uu02KJMjCn8Eu2ielmokn9GF0ezNIz5uOtTX5ZiZiF\/Urxq9wSyHd2EcUUCrXNIylaZfA06ORjau5kfZKm\/C\/YZNOWcezG3OupLXG8ICHQ9bx6lW164euD4fMmlamu2e9dqg1BbCEHjI2erQd0XP72dqPxeKRxpylhR+J7INJe173+rBcrhnsA\/zp\/KgIPrzB6p8ADwXHKkOK6CBNpKLXmWuScB8O7\/ql34qHNgWzQY60\/sWifpuzhhfPNtnA2Xl\/Lrlt\/Yzy+rAX5zpU0YFe4AKVKlcKgQYMwbdo0dO\/eXTGw8xfmZMBPyFLaXS+oE899Grj\/W0Q6OYUSKJz3aagHSPSR0P+U2rILL05T5JvBjrKk0brUdTTLYoROKBOW+WLI+xBE+N\/A\/bBYvSkJU8TJjafOL+GELu8k00ojMmFzVnuzxkmDok2Vc\/bmxJ4RRjossTIWUyb5GIybQv0VKGoH+0W5dBfTdHJ7uHEdO8FVLZkDjtd7yop1wgXKTfam9fLSA3ae+\/OkTUrZsuGAXb3GJYx\/bc3CDNMF44oNF3FnxgC8PrmVOVCNWNiJ5TuljWz2saYG7ez5GOnkVvDCZBzZu1n3HhANQ5s8xWmPeu8oe2Mdlesn\/NF0ig7Yuf06ORQN8k+LZU09FZ3CaM09Rj3C1r3RbGycnpF7ASvnW03KOhqwK5Q2XZheuHAB8+bNg5zL0549e+LixYusFwI6rxMhjM+b1r4IiG\/l0e\/I\/KtVo6e4mjoM2xvdlT0qKR5QTcdIDhIO9V\/A5ex1lqUmbOFa5oxTynMxmtt+RZl\/omTHiyELFt8n7\/VOKCSsua3TY+C1CkbD4MoVKl2gvtgZqYh+mu77G5rmfYeojBdQ9bidLk+noT5zulvDeXsBOIwLxj5ROkS30n6IyfEz7FzumAzaJQR2odcm114pWxOlf5Pito3JI\/\/G7gi\/1JEBJ1cUYoc0YlTMoxtn2WV\/+9\/bM2DPEL4W95fMNCleQ9ZgFTq0QU7HAtifOYvsaJ9dihXH1YVfYNXgNouFQ4DO49g79dwO5+wTcKfzt+QacorY5FEcaVJOG2qp4+HhAfpfQEAAGjVqhGfPzPPvsMR8VBmPvVmzZmjYsCGGDRuGR48eyQJ2Es6SJUuwdOlSkIXInlR1ceJKfmRKfRKpajZj9r8UspWO7vXu\/IXw11MhjIJoTLikYZ++FY4TLmnQoEMWnaZG2svH3qdh+3Uunk5erWh9xO7\/pGXJtQeW6ohzscXP9YPL0q2427cFzv+9H5SzM1WJhbDLVB0bBvYxaeLG2yZgL3zmNUo9+sK0Rvqg6f9vN9pkMqiWeHwkvzxL6jOKgqfXo\/lK5ck0JcSNLQpiX9V+JvOd8nY4jy3WYPnvdAHufnkbbp\/8lpHIVPFp2xRBmQfpNFji18lzsvzL90yRUKp58pMRhbgg5yeKWS7k2Ok9obsNyndKeUyFFj2GxmoI2Gn9uvf1xMzctrK1bGEUSqLB+AUqp56UrqFQQyfF6Gf3gljUZaVZ74KptbL07506dULnzp1ZNxqwy5A2cesVKlSIVzM0NBSNGzfW+zs\/Ck2YMIFp7HzXFF52Es9YfMEF5EhRjwG7T6VJCCpbDWkm+MjSYDl1UmvGU9SwsmIgR9rLqdBPsC5ogwONMsr+UGjwdDFplX8G3lx4q6MlCOgopVrU3s2KNFghgJHGWLdARuw8WFd3QqHf8\/SfjojUFeG8qRHzdJRThJoUAULT3LbsscrbnrDNQSqOuLF2CdjDL3XSUQAEqJEB\/nKGoleHTmOVFuXB4RUL0Pbap3hxU8QNGjtVcW\/RuStDZPPEbTMWxO6Oy+Ntwty6xvpxe8XrRxeU5AhGJ4F\/Oi5HoanN9Th27kMxeOx+k3cKNH8KfObrESWZxENO9EehDA3V59TTvl6PZX1DwveUrGlorkW2jGBesko3Q8UvjYUeIG29du3aDNw1YDdDyHKoGLFghc414jColWo0wnWfoZAbCIxrJ0IbdgI7inUhFY\/GjCmyR+jyVKldNz1HVFNcrXQ48ChUkl7i489xeR3O7J4ha3j08ZG5HLf+4B6sE\/59hcvL8iXYmYpvcEqTlRBH793qIJw87eA\/sQ\/6bSphdD6GTBTpIdqcyYZdSapEQ0BH40rvY4+PB3srSh9Ia05OYFk3FcC967uxp99eWDt46EwgeVq8zZdyo8fZjCY3MppXsU7jUMzBRhLYaZyZIkNwc\/d4We8BfUd7++6JF5WRA\/uhl18VvbOk1NCaU15emjcVpXdUsgaeRJU0jj0BgjYH2IUfoBjYefyUTjussO+5aV6MWxpIHTvpMo9+l\/uhJEAMBh8l6ulM4Vg8exKBoJhpcEuTBSNeHcC4CWHMmYVHMOz9MBe2Hjxhcgh0QnHsuU3Php0\/RJopWaVIxRE31LAhM07SRL0GFFMUksFzyG4EvJkJJ09b1F54VVauTAp6JRUulm94cmOd0PzIyYaASXw\/Yy5XTMAesrUIy7vK3y9hSIGBDQoibYuOoHynMdt6mbxToDEaUjbovU\/V1xPb38bKvuDlYQUoaxin0agPqUiiJl+s\/6tA3yZtDCFOmdHjyxtFGr\/cPpKqngbsFpK0lGAJyKvvbYZMwUGw\/60znEccR4HgzDpth1Mr9XfOwOzpHY1aVXDN8sqi+\/jQLaeexyFLju3ZH9bXoqDEwob4cPt6o\/WoCM6Rmxt7hjvufC7hgvY+ESh76w6LF0I5KLklQ9O8I+PF55ZaFn4RS5nnKdsNj2VPdbmFjRJg9xpQFO9TBcbbDOTkIxWPj+c7fbz3suzNlJJBZMwUHU+D5cDusKoLSywtp5CVDcWLETpuGQNTOW1SHVIQ+OlICOz0d4rdnuH3lqB8p+GnT5ps0hCw83g2c7aGy6aeOLDzKJS8c9L8SaG5traHyfFIVaB2rzScoHcqNKuh7\/yQBuwWWgBDwF5gcxdUThuJFI1rMGB3vfNVF8mOf9Btr8Vg5o5yJkfm3LAC\/H5+jqEbXunFjaaLsyquVug60Eu2Jx91lmvlCKQ55YMas2oySwjirCl7OwXZIp79QZ+lsmKNCAcudLQhRxVypxcW4rTxYi\/a3M5jNNYIPUMf7MWv7+JtZETHzCpqhx4tXBTZ7vNNS7wZmMOz02Y6Nss0eESUg\/35PSbnQvMxFH+cZ5dq9DhO9qUyrRfFXD\/27oUOiPn7RGEOyEJErkmpoRdPvOGRVl+mSC6Unz9NFhVDqRDhWkPyDkNuLHY+NtKsq5XpgDmlmum1197pK279U8Qsvwtu5hpxwtbsSKEmP9okqqABu4UEbUiwQvNE4qCpCDPO08cjJwMPPWfs2KnUwYKfFop2vIU6pyJY8gqyhqDSppKH3nFXrsjohDL4fQjubwk26E1LIFo+NifaP3HWy38p1QcHYjH1xO8VzI3zQqBIhbh72syoKLVlp3WrkP0dnHwKwq6ddCx28ZxqzM+Ke+GxelwwnWK2dXZnm5SYZjAmd6n15he0te9VUgx0\/H0QBoATAvvs1llwo\/JY3LhaQtZlMY29YLUGeO47nDnn8dC\/9HdzTig0t1E5f8KQFtP0TlyB6T9hyf4SimTH5UrvT0hgmO4ugig5KuTs9KMVDdgttGJiwZLVxMlzL\/W0NGF8bj6MugcK4ItHR+wp0N\/kyKTCCfCHzLVlF9vzSsWqNjkwQQX6WOp+cMVDv62oODMWh3ZN0wsWRZpu6M1oNL76DVyNFR5OgMBGKmE3AY8S+oJ52fpeRouYGARbWTGzQLKqIXBXasv+uOkOWKcfjK05y8bLd2poTm2HZoRH\/nQ4NigoHq1E2u37zU1kb6hS623uO2BovMLNjuc7vWedG6FT6si6U+ActtjahFLuUeYpJXcKNMb2U4rj2PsXuo2RbxAFV3TCqa+3E3xCMfU+qvl3DdgttDpSwL4goB6qbO2B5kWeIqVzeQZ0YtdzApSMubqg9ZQ2mLKroVFvPrLp3nfxnc45QzgVpRdwvP7qBa2w8sMdZrdMDh8EdpH+Y7Epd1mzzABJI6+Z8y2jnoIa58H2lRX0oloSv7r1j\/t6ZnSGloSAvVgLF+Qa9Dhe4CdqZ0KB1bj\/7zZZuStJI3UbuAyd1uyBy9FF+CunI8o\/jGIUVDO\/VixAl1z7\/cxp3XBhZW6W7zTDkQlsDHIKeQ2HVWyMtKsa6QE7txGP\/KO+ogu8yrM74Fp4D50Gq\/TUZmzM\/H6Dc9q0rnVShSOub0Os8XKWM132LonD6fJ7liad3RVH5yRzTDJP5BZb\/ESXf05tRkn9l4sG7BZafUOCFTppcMck4RDog\/F4dA59PtQ3yVsSMA3OPgGbxj3VAwZqT6nJHI\/oN3FRG9z2fqrLmkNtcfft9CkbKTIho2eF1BNFt+RhivmcmSt+UTv4bk\/D\/mQsprexEwrdKxDXPm\/uY9kXcKThT\/pnOIqkOQHP\/OmYgxdplRQZcPKn14reDDp9rf7pM8rP24oLR4YajZvCG5ZK8MBt2O0W3lcUHI3a5PFT+IbETR3t7UbJtjYRT5qbutL7kbVbETyb6sxCDFMhxcLPqhHsArxk3Slwp6f2p1uxpNi8mHuyEPPy1M6AnzMoDicgnDOdSiguEt\/IEpqLQNFLlIiVNWBPRGEKm5ISLF2K0QdbP86WHfeJnhFrsAW6BeLrkyeyLCtoE4i5usOghqo0LruU1QJRJVuWBxtMvWZIfHxutaPvG80KxR1o8i1cAUrk3Gj3t8BqUoU2AbKlFmf2oboE6k\/L2bP8p3IDlpFGyx1ROE9P4LDtzRtM+S07HnfPLKstOu3QJkuWQ0UiZuLeL4GygL1MR1d88HfWcyKjDcqljhMCKqaX1bdQTuL1JmB3q\/6UvR\/mROfk4MblKeTYKd9p17hB8L2ZETZnrsoCdp7pafTTQei96rBu6OaeUOh9Xdhlle7CmNazeXtns4GdO1z9qGBuCn8sBHWKmlVlSAElMxACu\/XbcGydUASfTz1B9uNh4NYOPHUYjxdD7ZOmTaEGpsQUMmkCSB+yX72BBoFdSVx2Dmji3I+0GREHnmpXaLx41cbkcWZgDqRrVwAd3j1GuehW+ILOLCnzpLsV0KKBt846hjTUDj\/lwZTe+3VHepIL5QalJM\/CYiocq5JQt5yvH1vlrJ6HIQFr2TQp0NV7Bd4UDZKVnIJik9PYdr3qZDDfqZSsaviUwIUaC+JRPtRW2hxvEDpukJJXjjmSpfaw0WnnSr05xZ3xXLD870KO\/cLcqugYPcRovlNxezyCozBuDa1\/m4oOii+LqW1hWAH6N8mt750TGHP+T0VyS46VNY3dQqsqFGxWRGDLhMLYk7U0C9nbrdZLOB3xwZlV+heJNBT6OL0aVobDh75oN8FTL1s7Zd\/pNew8u4TlVgv9xwczG3ZxIQ00X8uxOH37vSztnz6KJrbhqBE2S6890oTJgoE2CZeSZ3GvXfwkzeK+OW\/a4rgt7jTaxO4UyAwwbuUJ1JuwCCfPvdBZAlH7HXJUw+5ho7G8ZUN2CdeygTfINJIyTAkL3T9UvvsRswY9llw1JU5KXEtc+LoLmu6K1OXNJLllfRzNOPYTKW10poPGXhN65o+AuizphJIYJTw356Kg9mj81w1dF2wt7p3C0rMrFb2ddAJ4XnI4Xh23ZZv939cP44m1teIwC1IAL0xAQlYipL1PWTcIJyf2Yesmp0hx\/tyGnUIUK3Xh71rARs\/bmMZEVGLXCNPpBI2NV5x8Rc7c1FZHA3YLrYiUYCduH45ZpxugeYvUKLwsI9Zt6h3vyM4B58rChijgVBaFK4ToQI54XG4ayS87P+2ppGc5QWFQ3cv4MR501bSsso\/0RCXUjQ1D9M5xeqZoXDykDXs0jUXMz+NkX+j55fVmwC60VBHOgdrmjkXEC1dMV0FHxxC4c42dc\/N0QrnXJlu8jYyiIdpn8cKwzEcR2MJFVlgBLud2t9qxiIXiQvcKbpe3ywbFLT2qokuGyRgSao3l23uZDGFL\/YljkPMNkUAu5OlHxS7tdAeRu0BtZgL48epOllWK6CElZpNSnwMBHSUjefNls15WJZ7v9MiqnbKoJwL2B1MPMCeqIvdX6u6FjN2dGPs8ufzyTH\/ANtRZt45h395iiqKjitsn+onN9cI7s+grC8GJ4mY1YFcsMnkPSAmWxzWhSzpeSAsXarAc6CgQWIRXVkzt4cqqCm3d6d+GbLopMxEVihrJL6WcnZxMDpo+Ugr4NPDQR8m6pFlRJh6yb5eTwIA07lrdsrJY7C9\/vocHd7\/ZxIsLn4f9vLrI5lApHs9O8iGAJ8cmQzb+tJnRvIM80uPLRR9ZQMa1R\/906SSdgIiCkmupQWtG4Qxat8iEjNe8MLd\/cZNewyQHcUx2HmLhj6J22GplpfjCU3hnwKk1ygWbWJwx59hdH\/2DC8vLMCsg8j+Qe1lMY3LssQ0+aW7rKRDmJJgRboyVeq7Do+gwzDo23GwbduF7qWnsJuHC7ArJimNv4fEGQ6eUxpRut9D3p1i9RAhiDZZf4OTwOMw49unvZmPkaX2zR3pm7cDT7OJPnKdRKHH60L2fvWEx0E2FVTUWU4QDzpa\/n8Mq+L1JLZbXL3Q\/PfJMsUO1T1OR5W8fzF3VIJ4my7XU\/ttfYkNYLAscJtzIuObOqae3Xe\/hxuUPIDAn2kZcCHwoB6fNii04c2WdwReQh\/9dv+KVZB0C9u7NXRCxJhynlhk3naMNe7rPeFjZ\/47WX6rh2HJ5HG8V1xR4t7MQKETC0nUv2UnJ3IxHNAm+kROlkSU2lsVhN7RxmfNlco499bE32LGyvux8p8K+hH4R\/D0hG\/Yox+MIGvItM5ncwgN+\/XM1hmnX3EpmoZ+LbPt\/uX39aPU0jd1CKyYU7MDc79BmuC+GP2mNq88zoVAdH5To91jSnJEDu7fbWmzIVoDFar8R8a\/O6oCsTeZPKoGx\/z7EvKdRzIY90iETqm7YxrR0YRnWdhsWZ\/aE85m2uHPwFprnGgA3u\/imaaQ1Ex\/vuH00jt66YFCzpjgyZOduKkAST6K8w2EjFjvbszuFxQMnsHgiBNTCBCPUGZmALhoxDteHjMS8t11wZUFfVJlXXm8cwuxOFPSJEpNka9JSl1aQ6JpmJTJgRYFtKFjZFVVGXNKFaqB5H366Xk+Lpg3AJ3QG9q4wDP6cR15+Z59R1\/6+Rb1RYHx7lu9UbshlmhwB09veO1E39TE4X17KKCGhk82RUOXpCCnJ87a90ai6+blZWbSMfQ4c2CmFnEObhWhQLg7Xb50zeckvbFMck52srvqlSRHPOkjuZ0nzpUiOX8YEMWBXmlBEbj8\/Wj0N2C20YlKCJaCof2A+uxQjwO79oCvcI4KxY2txTB12FxvufoC1gyf7aKovrYevZVJj7qCKKFbjLzbKUCs3eH6JBawi8KpoR3b8t51Th\/27\/slLWN2yCzK8voayedLhxP96udbPngPnF9vC98RnbB1zF+XXnEfK\/stx+FkAay+vVx+8CjuG+163karmEdR73ROuKaKxZtVdtlkU9XiP4ND38Ow3jQHpvoUP0S6qLbIuuoLFvUfAKS4CX\/7dhqDb3y5vyaGJym\/5W6JJ1qogxx2bO7tR5nIA4tJlRGhKa5TJ\/Zld\/hIYPn0bi79fxIIy3hQ6NA5Hb51HQWc\/9PAegsU3miD83Xv2zNzBnvDLaAvrDpfQyz4tM2+jHKGHf+mGsFMnUD1DTrZZ3E77DEV2HQFlDaq4uCqjb9Yf\/oQdFXaj65ESTH5udlkYwFOflOTi2OlT8IwLR812ZbD9\/Fe8vPWQxQmhk0T948EoWCIv3qZNi1Udf8a7p8Gwv\/AZ6ZyPsHnSPHhZf6UqZn4MR4chy0z6H\/BnxPbbdEqYeeEt2h4KYY43cigv8evLtVb6+6vAPFCSEs\/UpyDMy8o32pYrp2PS8k6yqCdqv1PJdjhXuSmzIOJZjxISYprP92TKlAkydTQ19x\/tdw3YLbRiUoLN2zwA+2wPYNPjgxhf8E9mk30kyzl4VDyAY\/PfYHTaCVgY8gr+WV3h\/\/EAtttWxfWCG3FsTSC8M2dhbtfn7z2HdYbOWJimFZa6VsfpK1\/Yv6kUc93J6lu5XEDMxuvs37x+ZIZMGNR4O3LYWmFIwDfNflypyRgTFoW4x99yrFL9m+WWI2bL38z8MCaFNSZHT8XNQ69Az\/tc2YpUNXLh09572OA\/CdXcTyDjSzuMzdwN2YPOYfPymzi5eQaLYfN4qA3WedzEvogXWLD+JXa+HIQFOf0Q\/D6cebA+6VCBXTZesEsFz9gw1Ht\/GYvS1WDjcF\/4K\/Y0foCRqTdiZ3o7XI5pjD2VOjCNmAoF7SIQHO2+HNvjXHHpxbe\/0\/gp4JVT8FX8FfYVHV8vR5rOOVHv4iLUerkRhV4sY\/Wu1F4Mm\/cP8cuJSTjxxYo9\/3eP+dj60hevQj9i7ocAFCz6EDVeZ8fn2OfYtfUYDrUehq2uLXD8ryfYUXQ2XAqlwdCj1Zi2+qp4fZakxK3AWdQMnIyRi77RSaYKATtZ+lzYHcWoBGaLHx2LuK15FXth8r54vP8scXEIWFUYnKYwNRY5v3OOvfiLfRg+rTXa3+qBGXeAgbsbyQb2Bq0b44T3ECZzAnbabNkmu6ITzDmhcCcvAnay3BF6osqZU3KtowG7hVZWKNjNFT8j96gKWOu9GpUbVkZ67yDGs9OF5DmHnBgyPAsy1vxms+3z4RM2NgjHwMtpYfX3OTwdOQ6f9y1A+by\/Y9aDVez3FCnvMPfsUb\/8hbTLf2f\/pnLephD73S3de7z4EMz+TfSJ\/7mFuBt8AG8K+aG261ucOHSF1f9lah1EX38FLDkChwFDsblSc7h0XIibUcvxwNsX1dP2QpZXD3Dk+ThWf0B9T4z82h21Nw9h\/ff1yIMvcV+RqlBJeJV2RxOfB8xEkWzBiXpa5xrLgJ1oJZ\/fouHg8QvIBPL5Hm9MmRiM2cfjWLvHygYh+G0s\/hp2CC9vPkTcyJbs7yEp8uNZSmuU+HwNEcU64k3Z+rC+9g8q\/j4aj0p5oMpvzdCk+Ev0aHtIN3\/aVChhg8Olqex5h+bVYb9uNnZsqYYis3IwXr5pmWu42jQtPO33Iy4qFL1+D8Go042YvIi2SWuzDlGfTuMfj7Gwy+eNbAH10coWyPnam\/VT8tMN3K1bDfN+38zor+65UyN1obponuqorHynwleu9IgpuPUog87pijtsKYkTI2yPJ4wYnu8Eu+jmQc0S4zXnVEzuB+kxbGptRflOef\/ciWpKxvTsT7SRUTC065c\/mGWFQnHobcZ4o3O184h59tnk\/U9iyOFHaEMDdgutklCwW+u7ImefnCwWe27EoUDqavA90BY3Ik5KmtrRB\/R5TBCyjDzCgIMSP5NXpjCuDF3+\/W2bG0PzTEbqJ5dQ4WFAPMsZmpohJyVKy0cg93TMryjYLRBXrTPC9sFqnbu4lFiEnp+kKfGoiFT3s58LHhVx0qXno791RB8UKF0WpUv7oPHCzbBJOYzZjO\/YXE1n6UL1uNbl8GwDAra903G2ZO5JcdupEGC51nXEvV7vWVb7a+17gixmyjT\/Sy9MAY8Tvnt9r3hTICoh7NRJ4O+9uFG4LlwjDrN\/G4olLo7xTpw4L2U8U+HXBm5svv0rOSOq9RDMOZ5TcTydMSXaYH71X3XxXfj4o\/7sahYo8zuafLWuYMWrFIn6dnNgfxPsxxJw3BmQA5XGTzF5MS8chBTtYigXqpzB05qQJRRZdG3dG22WzOT086PV0YDdQismJdgiEw\/Aw8MG5T7mx4mpjXD91b86Xlo4DALjX8vfR\/GPW+Hf\/pjkCKmOo3NPFKq5HRUeBOB\/2rsOsCqOrv1aQFGxS1MDtmCDWGLUGI1iRwVFNHaNJRbsNLRxxwAAIABJREFUDcUWayTGmMSuxFjyWbCjsWvQLyb22DtKjCJiV4yKIv9\/xm9uluVy7+69u7j3svs8eRJyZ8+cc2b33ZkzZ95T9ZkTxvy4KlXcl26k5WmbMjkR3vlfbg4ukDJLfJPj8azLBiS82Iv472eZ9AYBXaXFNwxH3SmUQNkXf2fLhnzDRrE4PH2IVtedhkMJZxBR8n2UcHY1W\/SgUK+pcL31ItVBKip2Tdf4w28Pv9AKISjIDTmG1mR\/H542Bb\/eXJ2KKZJ\/AKiS0K0G59L4luyly9O3K+ODMUZNIPVxENNBdB\/6FS6+boYmh2MMOkuRZYzvRIlNQP4hpFRHpS4hpQD\/gMjZUyA9mtX2wuHaaw0HuSzliRHaRBMMSh1d99X7smr8KuUXLcrRgV2lUTHmWCEhlrBbYa42\/X8hsK\/228jAkqf28c0\/esn++e8idsJwW8fz2H5mj1G+DmH19fRM5ZkFUuqa8lx8Ou0qrGBEYSUCzmYLTjH993fdiB+ynjfJxc71IYra6p5VsWNG93RHg9MJ5IseiuIeudJ8wPiNHHCIjtcYtS+1o3GgWCz9Y+6iWWrI\/if4c\/XdVIySFG5q8FFp+C84hacBc1HF2wvu3aohZ9f\/YMmvA82JNfxO1L1H38tlyFkX87NIFmSkYXql\/yyVyYG9X\/ZfUWHicLbnMeWPLJI3i6lfogH4reo+w0Ena1co3BayNYeHo2ySOkt9ofX7dGBXaYS4Ywd2aYNNnXIheEEs5l1IMJrtQLVBhTNz2lDLd\/kF9o+MZQdvKAedTpJSOKaIU3E2IzxUwRerLsTj+zcJJi0wB+xKzJhIATqQ1M7PFVVaR4P4tVcficGOXq7w8\/wI7rGzsXda\/3RPJ\/KPxZ+1vJkfKNRCm5LCqzttKHp+hPoLBqHI5ZR0wYQfeEqPcpc+QFt9cqQCU1MOJGAP\/OUhwib9naoZnXYNHdMd4\/tOxE+VnqJGobq4XuaV5HqnXNhn3QoysrETbRzw6sZxdggrd951snO6VXqMU4nloZgu\/2SFc+8xsuqdckHiU6acgZJCT8LapRlhjz33oQO7SqPLHTtk4ECsb\/4Y95vWYlzsLSu8QbWjJdD7O5d0e6b4uevNF2mqDvGYMx3soMwJv9i9iOobaPL4Oo8Thx2snyaeTyl9szvXY9kpUrnHaTZOK4WXp7eyLI427+dkR\/x5iuMn23qAcpMpdXNpzVWoXjUfbp35JVUcVpzL3tY1G+ZfuIvwcbVwoGQIo0TgOflkc5dyHXBpxBxccKyEphM\/w5zTA9PNwuB54IsfjUareXvS+JgXdcgSNxN3TeSw8xtpRkkXr\/tJpFWzTjxLxcy57osP8WX+QZBT75TLp\/2FL6eeZzPYV38dZxkiSvCdqPFYc2B\/nmUgKparhpIDakqud8r1EdMo2Do9rhp+VkKmDuxKeNGIDGOONbVJRGXkOMsjW5429MW2Qa2NalenRg2c95uNfPu7IOZg6hqi4ht4aGJdxJ1UJ155O14jUiqfiNuU7Uj+owMrF0bATheFZCiO35aAfktW\/Ob\/I9pFhyEs5qBREipeTYrrIKRREIdPKPREq5Sfr22TnNt9rDjgVrMgamx\/nCbO7uXsyg4FySHrEvqU20zZHDT7p83T2vWLIfdYNyRE\/ASnvUvSxP1NPWLiqkLWMjKq9DgzscI8dgp9VGjji0\/G95O1p0DATs93pIsX+5h9lhDLNs+l0F6oaZu9ydaBXaURlQPsfAZLqYJUiIJS3kp\/HIgH8wINwEQbkxSOoTh2UtYi7BDTsw3BeH4xNbWtMXNMfVCUAhKayRKw19ruAJ+RK7Cv1TyY42KXAuzUpnTPoXjk0l5y2TuaWbqPv86yJMQXZVFQLdeAvG\/T7cxdYipc3p4fhydgf9ZlI0oGfwevbM3woMMRSYRY4g8r53Oh1RpdF\/peM6dahv\/OY+zhPqcM9U6rRkdL4mLnypL\/E1ZOxMNLgWyVqOUVSoY7WMEOdWBX0JlCUdyxM8OGYHbAM7QecxZz4x4aamqa6rZdzjeY0dndcDKPYs4E6Hs\/C2CZJ\/4Dh2NihRKSj0\/TC\/nBxnEomLzfwNxIYZjiebJh9KV7rAyenFOOJI8OCQkvirHTB6pF0G4c+qIonnxeCSPjY+CNzsi\/tmGaQzvCDWO6b0apFAzZcpfpRzb6DBmJPW0D2abxsG1bsOw3N7zYNwCJhw6ZHTFeher8tsmp2tKGp1+RrOg94j1JRGF0s2vHCkj2XJpmhk8x9gaRUSxk5NRlPTtJecTzI8n1TrliNIM9tnoyAzpiZCzgVwZvzg+zKKfbrGOsbMBDMQuK58OOD8cj+kU1yfVOedfiPZ30PpxWqprpb9eBXaVHgDt2zVfDMSrwpYGLvXvDHHg+KP1ZDnGeJDd1RlX\/fCx1SxyaIMALK74f39V6LrnCDgEdZTJQbJziw3RRSGFWJSd8kWUSouNOSaoTSvd59GuAJCMx+XbVSmJQqScsT51i7JyLvVGdFHwYkRVD132Y5kNAthJrJW0e17r+AB\/9mB0xN0+ydmQnJy4b1bwAfh1XXDIY01KfAHO810+pwk90WnXx\/ZZYVKWlrELRwjQ\/bgQV14hsXpDxqB\/7rjIax0\/HlR1rJPtRCHTjS9dnVLvJj29jxvZZuHxum6wPrUqPcBqxHNhp\/NsmxKLF+jDJKx+hML6CXFfmY8YZT5fUFVRG2Wrr\/ejArtIIih0bsWUCSw\/rX8URF0avSDcOS2A3fsEI1tbYhibN3FNKNEaeGjWRYCI9UGgWvZADf74Nh+uvU3GZr\/fNgd7Nf5OVhcFPNorLz1H4KIdvC3hHjmF57B1PfYlLwQskhU+Id8T584OpuL6F+tPyfV1tZ1T\/\/R9JXPAUzmj55g36nXyeCthptlix\/TzQ8XNrcthJNyFt8rXGNxmdAVHYbt\/WSRIXu3gGy4tMUA57\/9y5JaViqvTopitW+IHj9U6ffgNZewoknFI89zgMRdLrelgyylv2ijGj7bbF\/nRgV2nUxI41VzGeVw0K7HYAY8I7Ycz5XiCO8tin\/3K3c1UpRYxA6syqfpK0F2ciCEGl82g3bPDKY3VMl8fY6\/zuiYbDwjDCL4Jxsd8POIerd96Sgxm7+EYqgca8042MVoMSFh+RYjAt7\/0KZ8Gu\/rHsQ0BhJ9rs5GMgd6NOWBKO989j7J9sd2Ax9sYfJKFwIW+salFZMm8KlzUm1IUB+b2tD1BufinJKxMpvlCyDQf2VU2uIKLoNGS\/UI59jIk5U841oGEuHJlSmqV4xp\/crdkPmRybtNZWB3aZIxIeHo46deoY7lq5ciVmz56dRgp37O9zQtGp1RsEBf7K+LGlgApPAas3+W9MN1L2Tu6GJwG7d60eWHJkOQM6CknQpl+HhEQWspBbkkxsLKUYDu+Un8XY+w+4gNW1X6Dx5edw\/toLHz\/9BnF9d6U7q2Pc8hv\/wqznj+Fes2CazB0C5WnjikuuBEW68Swb\/mGk0BBx0RxbG8eKXe8JcpN1QpFXric7ifmRLh5jP9G4Knb4TkC2vKMwx8Uf4VN\/kvlEvZ3Bnq6enx2uIWA3dbhKtnAFb+AfuDMtq6NxwnBZ9U6FanBagWGlCzASNKkZWQqaYveidGCXMcQDBgxAQEAAxo4di8OHD4P+Dg4Oxty5cxEZGZlKkhjYJyYFYMtfDdE7XzGsGmsm99zdkT3wuZufwb67Kank0szYcYgHohKTJc+yi3RrhxSPYYwr\/E0+ByaPZrD1yzXBqVaT02yEmnOJsJwdAR39Xbf+aIy7dBEj\/uzL0gBp5v40cC7qeR3D2i\/6GhVJYSe6KBMoPepWmhkHjy+GFe6OksFYyGl+9enbWqJk72jH\/Fg8fDfyOE1A7Lht5sxM9bswDMHlUwPvdX0wZ7onKzohLAEoRzgnxqIQUa3XrzVLZMWBPeWDXahT8hmSy+dLUwNAit18YtKprrumP2RSbNFqGx3YrRiZ6tWrY8qUKYiKikozaxc7ltOpSt0kSq8MHAH7iibRSL66FA8j3\/Kqm7t4aIJmRpztj2cnfB5dw0Aba04O\/50+One3PkyVucFj7F1\/XcW42EM3d2ErFKmFD6jm6j8X\/jJaPFruCoX05MRiFK\/mFz8MJPUwlil\/8Bg7hSLmDLmPiPLP0HLmFoyOfMsAKefiutKZAzofINRZjhy12wrz2OkZ8LvUAzvDN8raUyAd+enTsDqu+Dn6tqRVrNq22Zt8HditGFG5wC4n7MEzWaaKiLksrbAjnhGbq\/kp1y2N\/h+UQjwc0f5iAKMXKOgdha0JDtjVeLckUUQE9tPMAUazI2hP4aOYw9h9SFqYg4eyfqp7COWfv10VyM1fFyotJNTi4RgeY6eVyeseAxkPfNX90zFvpL8ke4WNCOioKEZCsZwo2\/SUJjNiSF++arkx\/pGh3uny9Z\/K3lOg96Bm9yLY09oNE\/2OWB0KlO3wTHCDDuxWDDLF28mBPDQjFMUd+3pJD3i3cUdkrSj2s9TZ2Mb8yfiwjYchl10o2xKaU\/pQFI5egCvn3oYgzHHISHELveh0UU47rQoIUOtN+hujwipjTansWJPsiKZH\/iOpuDOB28CuRTHk5HNDrj3JprDOlAorJLFPCnXmKY+Ld78tIkL2UpgjYnfVNKsNc7by07sne73H6GrJXv7xKDM8EfdqjAVRGMipdyrskz464wsko3jyG\/R64qBZ6lkKxSTNisOO4LIM2IvkvIwL4zuac5\/R3+kZvlM0J6MWlnOGwqLOMtlN7u7uoH8oRBwUFITbt9\/uC2nh0nwxa4qvd+jQAeY2T1PWj8X7wTnR5XB3vHEujDLRDzF7eUuzPqalrlPU\/VSFlHnh5+CerrIr7BAVQKXf\/osdO6axvuvkLoLzg7ZZdLSeA93RRm+LPFOcnYdiuv3Rnp1AbZgYwrjYP0upjKHj0ufF4Y4gYF8y+WyaeD8B+46erog+8UwWcx8Be6lilVD+e3+WcUIrlgW9luOKfzzufD\/LIsIpmq3ziw4SZSk3Ew12f4bOYxuzohPpZTGZHez\/MU7eyJZN8odfikyl2xCwx\/6QzA5sUe3b5fFnLU7L5Hz+UpIJlLbD3uX16NEDPXu+raqmA7uM0TYH6iSKz9gpBn\/ixAm49F+BnOW8UC\/iDKZve8sxbuoSFrUQtuNcJ0k76koGJ16fMqTLaYy98LZqUWfXiljYaxkLIZxuttacOql+N8xgW17Ey9tJ7DcO7GOO7GO1TnvmeSmJi50L5hu84hOtBKZ3r5+TnRJHH8bua+NwM+IOVufIwTZOx43ca6A6lmWwkcZek\/2R+HwiOy0aXuIKJjgPwcC9N2XxplirQ0bfz0MxuLuLbe5TvdPB8xpZrAbt86RHrWyxUP1GNlv39\/dn4K4Du8QHwlQmjFCEOMYlt2AvB8pxZzulYmXkOd0P5wXKfiko+4KWvbSBOicxEV8NLmVxdXihrfzDQSB\/fe1grO55Eeud\/wCO10ZE728keTZv4yA4Vh2NLdeasvATv4QHgYjWVupFwON7+BFmjTjPZpUU7rAmrZPkEa0s56zn2ToLtzTC89q9Ubvjeyg09r+SVmNSbdBaOwJ2KgA+qzdYFlCXMy\/x7ZZPtKamro9gYqkDu4THoW3btggJCcG6deuM5q4bA\/asy3qzsng3PJZIZickORSzrlslN5pE3DEc2qGMmPg\/HljMhkdx9j6Lu6DmvRtGWRcluCDdJnyDsdsSwOPNHUxoVgBFawXjw1E1Ja1QCHg3eGVFkr+HIZedb1BaEnoiRQmIpq8eiaonNrIZ\/+WWLijay1Vy2qTQWHHRCuFZg74FgvHY7bjseqfW+Ptd3EuhGJeIO+jTPMiieqfvQufM2qe+eSpj5MWHk\/itBw4cQGhoaCpJ3LHZNg5EzuCKBi72mNBvJZ3UIyAhvpiooMsGuRRvzlslD+ZVckr1\/6WYQDPOm0dWsNjor3Gncbbbj7I4Ysz1wfnY47IOZ017ey7D+\/XL4+xAP0P1J1MyaFl+IuUZAt\/kNGwe0odsWOkcaD2smEWHWHi914JRE\/C48yIUrP0COVN+kJz\/n56+FIpy79cAcQdHYlzsKDwMaIaN0SVwaoH5EJs5P2r5dwL2W5s+wIssA9GoT0lWDIUYR\/VLex7QgV2lMRE7Vm4mi7FKQHRilIpinHa6h7hJI2VpTisAul5TkezkZAaexvLRpQrlcXZ+apCHjrZcv4t1sXvgd2ihrFUBzYBzBa1juflUxINf4oo7UvUz1o50pFKCll78gE7h5gXwxsEH96+NRZ5N7TH0P8VYameB\/h\/I5k2xVJd3cR\/PY78XE8UOFgUN\/QKhB9MWDX8Xuul9pvaADuwqPRHWAjvfJKU0upV7DhpSAGkW+uHxjZJzulUyzyCWAJnCG3RRrP2v3V\/jashSyVzsXBDdWyBkM+qN8EuVaUGbyFIKYovtFG7wEgjLic+n5zMivnp5+xWLs\/MwUZNsU5BS\/wRcSj9Dnr6u2Hpohtouf2fyKbTV9PLX+LRreYw9VQP9tryw683id+ZoBTrWgV0BJxoTQY5dED4BRX8fgZxtKgJuEQhwdpaVo0yz\/KstijE2wrWXX7BKRcZOVMoxgV5Oood9ODdAzm0m21LYiK4izQtg4Q5H5Onihu7Z4uGVHIqzzedKPplIm7tuTdzw2icf21cg\/vSe+b\/CP5We4XqfwRbpK8wuohn3zcV3LOY6p48Yr8vJY+xhEXdwM2d9VHR9gk\/H\/rvSsEhZjd\/E67+6VOvO2Eez7huMhEMHNa515lRPB3aVxp0cO2nCBFQ+OQK7P2mE4Xu7YmIeB4SEVZLUIwcOzhdDYRgqQrGzTUHcyJbVojgxvZjnBuXCy7hXcCp8UXK6pBSFeShm2\/3uWJ9QAEtrrIR\/WTccH\/yZZGCnzKHZw2Zj9+Ix7GDSH3FJiPPNgUstXSza8DSsBnybg6foKUU4xUNli5MHYrHHcBw8ctdqKmApfn6Xbej5if+1KfLH1sC+aZ+inH8LHdjf5YCY6FsHdpUGRujYB0WqshOLvZAVX02rKrlHvvnHCylzMHF8NBpxRgo1mxMspJ8VHpM3d5+U31k5vxo9sPXRdQw\/8gv2NRskmxTLY\/zXyH\/sKXjlIzqQteXv+7KyiYS6CkMnUmyQ00a4B9Loi6lwfFoDMZE9JX\/E5PSllbY8j73wR0dZvVN731PQit8t0UMHdku8JuEeoWNL3rghixCLi2\/SJAy\/ZXcwEGNxMPGeWElWSCc9dQn4lKqtSR8Kt44V8Pr3MHwXMFMSF7tYL7dBQ7Bo8kQDXwznxbGUNZF0yuHhKOvEqoShZU14jL1ZwR\/hVL0sfErkxx\/1A+we2CdnmY87PhWx6lg9eKyeLinDS6pP9XbKeUAHduV8mUoSOXbasL6o9GIhHIoVAKqtkc1iJ6YV4IyMUhkTVTLNqFgO7JtWnsOXT5NxdogHmmydiomTW0hWQ1wQhOxtNMcL++6lWA3OSheJ5qGyeavvocij0liZywmj5u6UbKstNmSVuK4B0c8Gs3qnV\/r72KIZmUJnHdhVGmYhsI9L6cdehOLbl2PfL9KzJgjY8x99guipcUxLvyJZ8Gyrj0U53XS\/MFOEZIvL21njCspjH+zTiBWc6HyjFs5WHY28V76VlefMgZ1\/uOjQUqEtFWTTChuzw9qNU7FMvnra9f6X+PzWELzctgoXD\/5ojQs1fy\/5MOHP7nB2aoKK9Ypg5XuFNK9zZlVQB3aVRl7o2M7XrllUqahMBX88DJxoqH1K9LV0yOb+ojEWbXzSLJPy2YlLnTJYqFi2UhePsUfF3kXo5q6YF39VMhc714Fm\/S9dthpqn5LMg1tmWEUFwD9oxHOu5MWBvRdCsS\/wHt7fXhcrvvxByS40J4vnsefwO4yKBWvhfN8Yuw49aW4AZCikA7sMZ8lpKgb2T169kl2JXczAqLUcdqE\/eCimSkwn+FRdLouLncuhFcXFFTEGwi+5tV2NjQ99yHK4Oyi2l8D7SBVjDwqD1\/H2WNNvtJxHxOba0ubp1OIrsTnpE1bv9NCyxrK52G3OaBtVWAd2lQaOHLuiZ20UdjmO7I+ScbDtTtnALo6pUw475bQTqZXWLgL2yu0KY+HBV5jXogjOP8mJV6OqyD6wQymP4f8jKutdwQH7RoZaTLOrpo+EMXaXxGSMi8mHpfv+pX9Qs+93JZtCMYPu5cTPV0Nx\/mYiYmd0fVeq6P2a8YAO7Co9IuTYmR1qo4LvWTQ8OhoFXTyRENpV1tKVx8SdWgHRcafxQ8e8WNLGQ9EQilLms1DMx4H45UEPNEoMQb5iVeC5KVr2EXsCj8vLp7ADVIvzJiFi94eK7gUoZS8PxSyrvoQRYlmauaOUPhkhh8fYu0YcQ+H9CzXNHZ8R\/tByHzqwqzQ6Qse++jwKQblfYtOU9rKXrr0ajsLGaq0ZjzhR2+ZK+sLqDBE1TBbmsfv88xhFH94y5KPL6a\/82r5sg45qiVLuv1KpnXJ0kNJWSCecr9MCNDl8FT\/vbS\/lVpttw\/PYy026Cp+Tjex+T8FmB0qn7VVv6ITAvu3CBcaDbkkIpZt3Q2xuMABvHsWhbUIsdu6cJpuHXT0r\/5VMoRivPh\/g89s1cch9B\/LuDca3oSNkd80zYzy7fI\/h8TFYcmS5Ju0VxtgrhtTE06krsWXhANn22tINBOzTKqxH2LnW8L\/0D6jeqX5p0wP6jF2lcSHHfhfgjfJNniDLonMIn\/sW3OVetPx1H38dTlH3WGZNQN68ckVkSHtGM9y7KOYuuoUWzQrAN6UndnT4XvYKhQM7fQRP\/lhOkRx2NRwgjrE3\/M0JFy\/EqtGVZmTSszg0pwtC1nyFlJM\/4f62eZrRTVcktQd0YFfpiSDHRjYrgCt+Pui8uyOqPozBzvntZPdGB2uCYhPhNf8m5i73ZWEYTkQlW5iKNwR71cfM2iUR92ojWpTcblWhY9okphx2Ko4R27eYxaRdKpoLHoqZXnMrI8TSashISR\/wGHul2u1xNWImzsz6WknxuiwFPaADu4LOFIrijh3UsiW2X7ggm9lRKIuArkIlJ4z6uZImNxJJV755Gnb4NLw39bOY34VvGJesdNSi0JVKw5lGLAf2sObhWJPtMnK1Lmf3x+spj\/3Pb4Yib8c\/4Ta1GjZH2feBrIx6ltToRwd2Nbwq2LyY2aIFFl6+bBWwk4paL\/zL0x2Ds9TD\/DwH4DW6CNbs2yvbu0JqXNk3Z+ANPMbe6eNAvPaJRVz7Z7h+xb4pbCnG3qrkGWy85oNKm1diz7lZGehxvSs5HtCBXY63ZLRlfOwNsqFUt+KMi91afheaySp9elKGOWab8hj74KT8GJ39Hsr+3A\/LZvcxe196DZTmdrFYkXRuFMbYHbK7wXfGCaW70Jw8CsXUelwNy6P7IGlzfzw5d1hzOuoKvfWADuwqPQnk2EVBubC1dBB+vF4PeWeEyMphF6rFy9pRfF2rVw7f5iw9kSo+Ufk+31ORsnhxxHaRzVq2l4diPnK9gb8P3cGZVf20OjSK6cVj7MfadtG52BXzqjqCdGBXx6+GL2aXJaeQu1gtvPq+t8XArpKKiorlwN7j2hXsyO+Eh\/MCNZmmqJTRHNhf\/XUcFQp9jN9\/8FVKtGblUCiGioGXrxuPW+2f2fXzrNlBkKiYDuwSHSW3GXfsoaZNUTc21moiK7n9Z3R7vunZ0bGgRTwxGa2vtf3xGHto6y1IeeWAVS0qWytS8\/cTsBfN\/RgvHhZC9iWWr0A1b6gdKKgDu0qDWLRsFRzr\/QqOjcvgYLGlms0\/V8p8Dux1Lz\/H3vxF8bxrkl3P6IQx9mMJBdH7P2eVcqVm5VAoxuFya5w7VFnRmrmaNdiGFdOBXaXBq+Hqjk3TiqJizHy4xp\/DuSXdVOpJG2Ipj31W\/dvoVXA4TmdzQ9x0f20oppIWPBRD4gvvX2D3XOxkJwG7c7ZhyJ3ki80f2\/8KRaVHJ0PE6sCukpu5Y0v4+eHbBw8sOnWqkmqqiOUx9rOLB+BIzC67X6FwYPdsGIpSq70RuXC+Kn7VklDKY4+7PwR5979E1OA1WlJN10XkAR3YVXokuGOzN2iAwY8eKVKjVCVVFRHLQzEkLP+GUtgZvlERuVoVQjH2Oe0Ko1+7wngxyw2nVu\/SqqqK6UUxdrpKnDuHo5vtewWqmNPekSAd2GU6vnr16pgyZQry5MmDxMREjB07FocPp83nbVO6GLoMqYOPc1xHQNcDmQrY7y93x1U7r\/8pjLGHr7yDyDvJMp8k22tOoZi\/dofjyc4N+Oe\/i23PgEyksQ7sMgd76dKl7I5u3bpB+N9iMbXKeqJ+z8\/x06tAq3LYZar3zprzUAwpkGPDVNy6uOmd6ZIRHQtj7HlmN0fs0zsZ0e077YMdUFqUD9nn7LL70OI7dbQCnevALsOJfLYeFRWF2bNno23btujatSsmTZqUZtZOjl1dtizqvtfJ7jcSGZj\/74AS\/XdmIMQSArvrgkF2nQHEXxEKxbhUXoKX30Uj5uAFGW+O3jSjPaADuwyPE5D36tULixcvRmRkJAN24d9CUeTYzU+e4M8TJ2xyI9Hd3R3+\/v7Ytm0bbt++bdZLPMb+8naSJio8ydXfrIGiBjyPfVyrJVjq30zu7Wbbq62\/WQWMNOAxdsphj48\/YlKEFvWXY7Ot668Du4zRFs\/QaQY\/fvx4LFu2jAG98Gpfvxx2V1+OMfMa2uSylR7suXPnIiQkRBKwv67yAkQDkPu2s+KFo2UMkaGpXP3l9vHM\/QnKzS+FOSuTERD5QO7tZturrb9ZBYw0yDO9Bxz+DsDDuYFmb9ei\/maVFjSwF\/2DgoIkvb9yfGNN2ywFCxZMsUaAGvfKAfY6DX3xuNREhDR3h08hzZmiuHty3LqNOb9fQm3v91HW10Nx+VoT+ODKLYT9ng2jkw7Bs1dLramnij5zssxEAedu6PikkCrydaHKeuDEiRNsYqby9p7OAAAFCklEQVSlS7PALjUUQ85sVbIM\/srvrCW\/6roo6IHiycm4HR+P10WLKihVu6LI3nOOOZA3+bV2ldQ1M3iAQqhSwqgZ6TJNArs49GJq8zQjnaX3pXtA94DuAVvwgCaBnRwnNd3RFpys66h7QPeA7oGM9IBmgV3qAaWMdJbel+4B3QO6B2zBA5oFdltwnq6j7gHdA7oHtOgBHdi1OCq6TroHdA\/oHrDCAzYN7OHh4ahTpw4z\/8CBAwgNDbXCFcrcKtSJJK5cuZKdnqXLVHjJ0t+U0dq4FDGVg6U6ZnRYrUSJEvjmm2\/g4fE2HVT4bNiCDUIdbe0Zomfmzp07qd5FU++pGr9Z806I9RePRVxcHIYPH47r16+zbrSmP7fdZoFdeBqVjEnvZKo1gyz33gEDBiAgIMBAWEZ\/BwcHswNIdLDK1Iawpb\/J1VFqe9K9Q4cOuHTpEuProctSHTN6I5z6c3Z2Zi+gi4sLI5Pj9BRat4F\/lK5evcrA0ZaeIfKtt7d3qg+pqfdUjd+kPt\/G2on1F48F\/\/vp06fsndCa\/kKbbBbY6UtZunRpw9fT2EzBmkFW4l4h582RI0dSAYwwhZP6EoKP1N+MsV0qoTc9wNOmTUPhwoVx69Yt9hCb4u\/Rkv6mTinbgg1iHW3hGeI6JiUlscfv7Nmzhhm7qfdUjd8sef5N6S+WJ9S5T58+6WKQpbZZor+xe2wW2MVhAlMMkEo5S64c4UtJy9P0Dl2JVxxSZwJiegW5+qXXnh5KulxdXdm\/xbMTMX+PlvQXr5qENpriINKKDcZm7HwV6OnpqclniJ5z0u3o0aMsBMZXG+JVnlqrPmvffVP6mwL2CRMmGN4PpWxT6h22aWAXxvIIjAiIeNhAKQdZI4d0IpIg4pKnB1\/IUCmcWVIflvymBrCTXiNHjsTXX3+Nvn37pgJ2S3Q0ZZsa+hOw161bl600qlWrxvTnMXZTVBVaGgMOEhTWEMZ0ta6\/+KPE7UjvPRWvsoXvsKW\/WfO+GtNfKI9P1IhCgMJklupo6j5r9BfeqwO7Up4UyeExar55qvWXkqtPD93x48fZhq9wJmQr+nO\/C8GceDzWrVvHNvW0\/nESg4d49aZl\/e0Z2Llt9J7wzVMd2FUATy2HYsSgTubbQhiAdGzVqhXCwsLYrr8Y2G0hlGQsFMPtIGpkrdtgy\/qnB+w8nKdUuEKtdz+9GbsxUFcrzKQUVNrsjF0cetHK5qk4i4EPlCn+G2ojpCUWb56m95vSm6fiVE2uO4UDiDK5d+\/eBupkqTqask1p\/fkHVFyUhT8bGzZssMjPGWmDKWCfP3++pvU3Boym3lM1frMGGI3pL86EEcrXmv52EYrRYroj6cSX\/Tx3XehsrafaiV8KUzMjpX6z5kU0dq\/45RSvlLQ+BsZCMcJnSsv6GwNGqYkANJZKrKaseZ7SW3Hw1Fmeu877UMM2a\/S3C2AnI7R2QCm9GS+P99rC4Zj0PkT0\/21Ff\/EBJVs7JMYnCI6Ojmw4bEX\/9EIZahziUePdF+svPpzE343ExETDWRU1bFMC3G02FKOE8boM3QO6B3QP2KMHdGC3x1HVbdI9oHsgU3tAB\/ZMPfy68boHdA\/Yowd0YLfHUdVt0j2geyBTe0AH9kw9\/Lrxugd0D9ijB3Rgt8dR1W3SPaB7IFN7QAf2TD38uvG6B3QP2KMHdGC3x1HVbdI9oHsgU3tAB\/ZMPfy68boHdA\/Yowf+D\/PPtGO3OfMnAAAAAElFTkSuQmCC","height":225,"width":374}}
%---
%[output:664b6a11]
%   data: {"dataType":"textualVariable","outputData":{"name":"J","value":"1.5896"}}
%---
%[output:19e66aed]
%   data: {"dataType":"textualVariable","outputData":{"name":"b","value":"0.1577"}}
%---
