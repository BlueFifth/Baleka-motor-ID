clear;
clc;
% Initial guesses 
hold off %[output:1ef8beac]





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
%%
[J, b, Bt, Ct] = AK10lsqPos %[output:738e7998] %[output:6c796131] %[output:7bc6a56b] %[output:18a3e3a1] %[output:7926a099] %[output:5e4e4070] %[output:186d884a] %[output:53ed4466] %[output:4bcb810a] %[output:687d07b3]
%%
function [J, b, Bt, Ct] = AK10lsqPos

%% LSQ for inertia (J), damping (b) , Breakaway torque (Bt), and coloumb friction torque (Ct)


% Data trimming for lsq:
TestsData1 = load("Data/StepTestsRound1V1.mat");
TestsData2 = load("Data/StepTestsRound2V1.mat");

% Lets use 1:7 for steps, 11:14 for validation
% Which motor? 
% Start with front left
MotorNum = 8; % Front left
% Position tests start at 5s
% End varies on tests. For solver time reasons cal it at 10.5 (a bit longer
% than longest test) - might need to adjust
SimTimePos = 5.5;
stepsq = [];
for i = 1:7
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

% Starting guess:
J = 0.1;
b = 0.1;
Bt = 0.5; % Breakaway friction torque
Ct = 0.2; % Coulomb torque


pid0 = [J, b, Bt, Ct];

options = optimoptions(@lsqnonlin,'Algorithm','levenberg-marquardt',...
   'Display','iter-detailed');
% Optimize the gains
% set_param(mdl,'FastRestart','off');           % Fast restart
% Make Ct < Bt
% Ct -Bt <0
% A = [-1 1];
% % A = [0 0 -1 1];
% b = 0;
% Aeq = [];
% Beq = [];
% pid = lsqnonlin(@tracklsq,pid0,[0.1, 0.1, 0.00001, 0.00001, .000001, .000001],[50, 50, 50, 10000, 50, 50],A, b, Aeq, Beq, @nlcon, options);
% pid = lsqnonlin(@tracklsq,pid0,[0.01, 0.001],[1, 0.5],A, b, Aeq, Beq, @nlcon, options);
pid = lsqnonlin(@tracklsq,pid0,[1.0, 0.5, 0.01],[1.7, 1, 0.5], options);
set_param(mdl,'FastRestart','off');
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
      for i=1:7
          in = in.setVariable('qStepSize', Tests(1,i), 'Workspace', mdl);
          in = in.setVariable('wStart', Tests(2,i), 'Workspace', mdl);
          in = in.setVariable('wStepSize', Tests(3,i), 'Workspace', mdl);
          in = in.setVariable('Kp', Tests(4,i), 'Workspace', mdl);
          in = in.setVariable('Kd', Tests(5,i), 'Workspace', mdl);
          out = sim(in);
          simdata =[simdata, out.yout{1}.Values.Data,out.yout{1}.Values.Data] ;
      end

      F = simdata - stepsq;
      plot(simdata, ':')
    end

    function [c, ceq] = nlcon(x)
        ceq = [];
        c = [];
    end
end


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

mdl = 'AK109_LSQ';
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
%   data: {"layout":"onright","rightPanelPercent":28.2}
%---
%[output:1ef8beac]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAOgAAACLCAYAAABrydWMAAAAAXNSR0IArs4c6QAAB6dJREFUeF7tm69PW28Uh8\/M1NTEAjgE\/0DFKpmemGzCDJgK0swhEE0FaRYEjlQsLclqJmqWINCAw3SaIHDrMj01s2\/O\/eY2l59rs76Xc+55mpAM1tx7zvM5D+\/tey\/PXr58+Ud4QQACJgk8Q1CTuVAUBDICCMogQMAwAQQ1HA6lQQBBmQEIGCaQVNB6vS6dTkeGw6GMRiPDGCgNAjYJJBO00WhIq9XKuu71eghqM3+qMk4giaC6cm5vb8v5+blsbGxIv99HUOODQHk2CSQRNG9VV9Fms4mgNrOnKgcEnkzQ5eVl0a\/JZJJ98YIABO4SeBJBVcx2uy21Wk0Gg4EcHR2RDQQgcA+BJxFUxdSNo263K+PxmBWU0YTAAwSeVFDd5VVBeUEAAvcTSCroQ9DzFRRBGUsIPE4AQZkQCBgmgKCGw6E0CCAoMwABwwQQ1HA4lAYBBGUGIGCYAIIaDofSIICgzAAEDBNAUMPhUBoEEJQZgIBhAghqOBxKgwCCMgMQMEwAQQ2HQ2kQQFBmAAKGCSCo4XAoDQIIygxAwDABBDUcDqVBAEGZAQgYJoCghsOhNAggKDMAAcMEENRwOJQGAQRlBiBgmACCGg6H0iCAoMwABAwTQFDD4VAaBBCUGYCAYQIIajgcSoMAgjIDEDBMAEENh0NpEEBQZgAChgkgqOFwKA0CCMoMQMAwAQQ1HA6lQQBBmQEIGCaAoIbDoTQIICgzAAHDBBDUcDiUBgEEZQYgYJgAghoOh9IggKDMAAQME0BQw+FQGgQQlBmAgGECCGo4HEqDAIIyAxAwTABBDYdDaRBAUGYAAoYJIKjhcCgNAgjKDEDAMAEENRwOpUFgbkHr9bp0u1158eKFfP\/+XXZ2duT6+voGydXVVTk4OJCVlZXs579+\/ZJ2uy0XFxfZ97VaTXq9nrRaLRmPx6QAAQg8QGBuQT9\/\/iw\/fvyQT58+ZRKenp7K4eHhjcOrxJ1OR4bDoYxGozunRlDmEQKzEZhL0Hz1PD4+zqTc39+XpaUl2draunG2RqMhm5ubsre3N101i29A0NnC4V0QmFvQ4sqogq6trd25zNWfr6+vT+menZ3J7u7u9Ptc0MFgICcnJzKZTEgCAhC4h0ASQYvnub3qFj+D6r9V0qOjI8KBAAQWIahuEP3tErd4nnzD6OrqarqK5iuoHks3iVhBmU0I3E9grhVUDzHLJpFe4upLL2v182iz2ZR+vz\/dMOIzKOMIgdkIzC1o8TbL5eXldIOoKOXt2ywPfQblNstsIfGuuATmFnQRqFhBF0GRY0QggKARUqZHtwQQ1G10FB6BAIJGSJke3RJAULfRUXgEAggaIWV6dEsAQd1GR+ERCCBohJTp0S0BBHUbHYVHIICgEVKmR7cEENRtdBQegQCCRkiZHt0SQFC30VF4BAIIGiFlenRLAEHdRkfhEQggaISU6dEtAQR1Gx2FRyCAoBFSpke3BBDUbXQUHoEAgkZImR7dEkBQt9FReAQCCBohZXp0SwBB3UZH4REIIGiElOnRLQEEdRsdhUcggKARUqZHtwQQ1G10FB6BAIJGSJke3RJAULfRUXgEAggaIWV6dEsAQd1GR+ERCCBohJTp0S0BBHUbHYVHIICgEVKmR7cEENRtdBQegQCCRkiZHt0SQFC30VF4BAIIGiFlenRLAEHdRkfhEQggaISU6dEtAQR1Gx2FRyCAoBFSpke3BBDUbXQUHoEAgkZImR7dEkBQt9FReAQCCBohZXp0SwBB3UZH4REIIGiElOnRLYFkgu7v78v6+noG5suXL3J4eDiFVKvVpNfrSavVkvF47BYehUMgNYEkgjYaDdnc3JS9vT15\/fq1vHnzRnZ2duT6+jrrB0FTx8rxq0IgiaC6eq6trWVSvnr1SjqdjgyHQxmNRghalcmhj1IIJBN0aWlJtra2pF6vS7fblePj4+llLito+myXl5fl7du3cnJyIpPJJP0Jg54hNecnFXQwGMi3b9+CRpu2bR2cdrstMC6Hc6r9lGSCPnaJmw+PrqS8IOCdgG506lViiiuVJIL+bZNIA1FJ9YsXBLwTUDFTyKlckgiqB85vs\/z+\/Tu7pZJvEHkPg\/ohUCaBZIKW2QTngkBVCSBoVZOlr0oQKF3Qx54wqgTRJ2jiw4cP8v79++zMZ2dnsru7e6cK3RfQncbnz59n\/3d5eZndBuP17wT0VuLte\/3\/ftT\/j1CqoLNsHi2qsSjHKQ6H9pw\/wXVxcXEDgUp8+4muKIxS9pn\/4tNzpNhrKVXQvz1hlBJkVY+t4r179y675\/nz5085ODiQ09PTG88+a+\/KPn94pKosyu5Lfzlub2\/L+fm5bGxsSL\/fX\/hmaOmCPvaEUdmAq3C+4sqo\/aigV1dXNy5zV1dXs5+vrKxkLbOzvtjkdRVtNpsIulis1TjaLILe7rS46t6+FK4GlXK7qJSgjz1hVC7Wapxt1kvcYrcpB6oaVOfrIiXPUi9x2SSaL\/hZ3j3LJpFe4n78+FG+fv2afUYq7gXkfwI4y7l4z\/0EKiNovlmhf8jN56DFjXvxNkvxj+NVRH3pbZfibRbYL469HqlSgi4WDUeDQLUJlHqJW22UdAeBxRNA0MUz5YgQWBiB\/wDl84KeI8FWeAAAAABJRU5ErkJggg==","height":139,"width":232}}
%---
%[output:738e7998]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Length of lower bounds is < length(x); filling in missing lower bounds with -Inf for continuous and integer variables."}}
%---
%[output:6c796131]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Length of upper bounds is < length(x); filling in missing upper bounds with +Inf for continuous and integer variables."}}
%---
%[output:7bc6a56b]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Initial point is not feasible. Initializing x0 at nearest feasible point."}}
%---
%[output:18a3e3a1]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAjAAAAFRCAYAAABqsZcNAAAAAXNSR0IArs4c6QAAIABJREFUeF7tnWHIHcd576eSJVfk1UtQjCM5105Eo+tS6AfV2PqQgINJm5IGBYMRiSFIVFKbouhDQdjCFWlr1KAG9faDrJZUdpG5IAfhYqyb+kJvaSwaN5VTS03vbYsruOLaRFKMY4wko1q20st\/k+ftaLXn7Ow5O7O7s78FY0lnzs7M75nd+Z\/neWbmZ9asWfMfjgsCEIAABCAAAQgMiMDPIGAGZC2aCgEIQAACEIBAQQABw0CAAAQgAAEIQGBwBBAwgzMZDYYABCAAAQhAAAHDGIAABCAAAQhAYHAEEDCDMxkNhgAEIAABCEAAAcMYgAAEIAABCEBgcAQQMIMzGQ2GAAQgAAEIQAABwxiAAAQgAAEIQGBwBBAwgzMZDc6NwKZNm9z+\/fvdwsKCO3nypNu7d29uXRxMf7Zs2eJ27drl3nzzTbdnzx537ty5qW0326nQvn373KlTpwbTVxoKgaETQMAM3YK0f\/AEdu\/e7R5++OGiH+fPn6+cODWxPvjgg+6xxx5bmlSr\/q0rGCnasn79eve1r33NPffcc+748eNRuoqAiYKVm0IgCgEETBSs3BQCYQQ0KR88eNAtLi66S5cuuTvuuMMdO3bMHTp0aOkGJnB8cVP1b2E1tl8qRVvM07Fy5Up3+PDh3giY9mlyRwhAIJQAAiaUFOUgEIGA\/eJXqOKFF14owhf687Zt24rafO+MVf8v\/\/Iv7hd+4RduaI2JngMHDrj7779\/6TNfDB09etTdfffd7vvf\/777uZ\/7uSJkde3atUIQ6FLdEgi+ULL7fe9733Mf+chHCoGly0JdVe1Tnd\/61rcKYWbl9Z1XX311qV9VKK199tmVK1eKsIwuC7HZZ5PuVW6P9c\/32JTr8cN2ZQ\/M7\/7u7xbMfI6+YHv66acLG+ny26q\/nzhxwj300EMFU13+PUy4io\/aqPv82q\/9mrvtttuiCrQIQ5hbQqAzAgiYztBTMQScM4HgT\/r+JNZEwEhg+OLF+NrEWZ64p\/G3Sb0siPzvqMwPfvCDpfCXffaXf\/mXbuPGjTeIF\/tsUo7PpHokVP70T\/80SMBUsVK9JoTeeOONm0RVWRCVBcznPve5on++YDKO4vryyy8XbSsLGInD8jWtHRIx+i+2h4lnDgI5EUDA5GRN+jIoAlUJoDYJ+xN9SAjp9ttvLyZSfwKc5E0wD8u999675HUpCxabsE1Y2OSrJNXyv913333FJG\/3lRHkfZnHm1Duc1X\/ysa279R5Z3yvjDHSvXxPlCXxWr1VHpayx6XsLTKmk+qwOuUdsrZXeYwGNahpLAQSEkDAJIRNVRDwCUzyGKhMXb5LeYL3xcikX\/6\/9Vu\/VYRDbGKtyispiwATK76gKn\/vwx\/+8A0CRiGwskcldGIuMzEOIQLGX81V9qzo71V90b+bR0V9PH369E2rkHwvmcr7HpmyCNXnZSFp4SITdMbLF1rlMrGSlHkCIZATAQRMTtakL4MiUBfSsdBPiAdmHgFj3gV5V9oSMGaIspCZ5B0pi4tZPDBWZ1nImHj6pV\/6pSLEVg5j1QkYP09Jdfg5MZMEjM8UATOox5LGDogAAmZAxqKp+RCYtn9IWURULe0t\/1uIh8KfqLXXzLQQ1rQQkrXPwkof\/ehHa\/dOsfZKTJT3S6nyBJmgmTcs5ffZ8nWahJDkTfITbsvesVkEjO6hhGldtqKKEFI+zzY9SUcAAZOONTVBYImATdBVHonyhG4Tnr+a5Yc\/\/OFS\/oo+l7dmUhKv1TGPgKkyXTnHw9r37W9\/28kjVJXIWtXfskDw6yoLGFvVVHWfSYnAJli0kqq8MsrqsvtN2gfGv3dVOM08Lvq\/n9Qrr1bZA1PVDpJ4eTlAoDkBBExzZnwDAnMR8Cfs8p4vdmNfbHzjG9+4YeLVBFr1b\/KqTAvZzCNgVOfatWuL8IkufxIvC5Cq9pU9F2WAJhwkgiYtK\/b7NmnDv0nLuv19dZoso7adeCftQzOLB0b5LSyjnusR4ssQKAggYBgIEIDARAKTEl9BNjsBX6yZgC2vVCKJd3a+fHM8BLIVMPYL58UXX7xhV9NJruDxmJyeQiCcAAImnFVoyZCQWd0ZTKF1UQ4CORPIUsBMctHrV87OnTvdkSNHCpvan\/m1k\/MQp2\/zEEDAzENv8nerlnxPCovFaQF3hcDwCWQnYPwkPJ0vo+28Lf6tl\/GGDRuWDstTLPzixYuc\/jv8cUwPIAABCEBgZASyEzCf+cxnChO+\/fbbxWoAX8BIsOiyc2bKf6+y\/bp165z+44IABCAAAQgMlcCFCxec\/svpyk7AmHHMRVsWML7HRR4ZrawwQVM2rISL9qzQBlhcEIAABCAAgaES0C7T+lGfk4hBwEwRMBIu2mgqN6PrAdRhezt27Miub\/RrWK\/XXO3FMzascZizvfy+aQNFCZlcrtEJmCYhJBMwuRldDHLtG\/0a1qspV3vxjA1rHOZsr5z7NioBUw4Z1SXx8nLlJdQXArmOxVz7lfOkkavNcu1XzmNxVAKm6TLqnAe08ns++9nPuhdeeCGrmCj96ovkCmtHrvZS73PtG\/0KG9t9KpXrXDYqAaMB1WQju1yN3qcHi7ZAAAIQgEBcArnOZdkKmDaGQ65Gb4MN94AABCAAgWEQyHUuQ8BMGX+5Gn0YjxythAAEIACBNgjkOpchYBAwbTwf3AMCEIAABHpKAAHTU8PEbFauRo\/JjHtDAAIQgEC\/COQ6l+GBwQPTryeN1kAAAjcQ4DgTBsQkAqHHAyBgRjiGcjX6CE1JlyEwSAIcZzJIsyVrdOjxALnOZXhg8MAke9ioCAIQaEYg5+NMmpGgdJmAHcMRslM8AmaE4ydXo4\/QlHQZAoMkwDtokGZL0ugmY6NJ2SSNb6kSPDB4YFoaStwGAhBom0CuE0\/bnMZ4vyZjo0nZIbFEwCBghjReaSsERkUgh4ln06ZNxan3CwsLN9ju\/Pnzbs+ePe7cuXNTbbp792738MMPu2vXrrnDhw+748eP93oM6MgahXVWrlxZtNPvpz7bunWre\/zxx92pU6fm6keTsdGk7FyNSvxlBAwCJvGQozoIQCCUQA4TjwmYEydOuEOHDhVdX79+vTt48KA7e\/as27t371QcOv5FV125UKYxy5l4efbZZ5f6qkODV69eXYi1e++9FwHTogEQMAiYFocTt4IABNokkKuAESMJk7Vr17pt27YVyMzT4nstfvM3f9Pdf\/\/9xeevvvpqUbaqnD6XIPrZn\/1Zt2bNGnfs2DH3rW99q\/i3O+64o\/i+\/k0Cyrwg\/\/7v\/37TZyrne1B870lVvWXvkcps3rzZ7du3b8nDYocI\/9Vf\/ZX7lV\/5lcITdeXKlaLMRz\/60SVvjf2bPDO6z6c+9anC6\/Sxj31sqbzvtWkyNpqUbXP8xr4XAgYBE3uMcX8IQGBGAlUTz10\/\/rG78\/r1Ge8Y\/2uvL1\/uXlu2bKmiEA+MTfJHjhxx3\/ve9wrhcfny5UKw+EJnUrnf\/\/3fv+E7qlyeD10meh566KEiBKVLIZ7vfve7hVfH95Dcfvvt7qtf\/ap7+umni1CVPrt48aLTcuWdO3e6qvb5RK2vCh9Vhbv8EJK+p9Caeab8dnzuc58rwmYnT55caqP1xeprIkqalI0\/QtqrAQGDgGlvNHEnCECgVQJVE88jV6+6R69ebbWeNm\/2h6tWua+vWnWTgCnnwJhHRQUlUjZs2LCUE2MeCIVd5IUxT82kcn\/8x3\/sfvu3f3spJFUWTRayevHFF90Pf\/jDJTEikeLXJeEgz0c5N2da+6pyeFTePEd+7o4vYO67774bvDWhn5kXpokoaVK2zbEQ+14IGARM7DHG\/SEAgRkJ5OiB8b0olpDrT\/iGykIqDz744A0CxoSBX06hISXHSqDoz5MSh+XRkDfFT6QtiyVfSFkd09pXl4zrh5UUMrK6JWDkZfEvEzsf\/vCHbxBSVcm\/TURJk7IzDtVOvoaAQcB0MvCoFAIQqCeQw8RTFUKSILjnnnuWckXK+TA+Gf+zSeV8D4sJGD8U5N+vLAZCPTB+vs4ky1UlHKv\/1hZ9zxcwVd4elSnn0lTl1jQZG03K1o\/K\/pRAwCBg+jMaaQkEIHADgRwmnqY5MJZ7Yit3\/BBS2XtjeSMWQjIPjCD6OTD+6iCFkCZ5YLRKyHJdrB261wsvvHDTv1v7\/BDSJO+SiTXfA6P7+jkwvqgz74wlHvt9sQHSZGw0KTukRxABg4AZ0nilrRAYFYEcJp4qAWNeBkustVwUC6n4K3KmrVaycm+88UaRxOsLGPPK2CokS4id5oGRGJm02sj\/d7995QFZ3gfGL2ss9J3yKiQ\/V0Z1\/eqv\/mpxa62qqtozp8nYaFJ2SA8YAgYBM6TxSlshMCoCuU48ozLiDJ31w1qTNvprMjaalJ2huZ19BQGDgOls8FExBCAwnUCuEw92n04AARM2QhAwCJiwkUIpCEAgOQEETHLkg6mwydhoUnYwAJxzCBgEzJDGK22FwKgI2MTz5JNPujNnzoyq73R2OoF169YVeTTalE9Lw6ddCJgRjqZcjT5CU9JlCAySgE1SehdxQaBMQMJFK5kuXLiAgGF43EhgzALmi+++615aseKGLcEZHxCAQHoCEjH6jwsCZQISLnXiRd\/JdS4jhEQIqZLAj956yz1z663uKx\/4AG8NCEAAAhAYMAEEzICNN2vTczV6CA8JmJduucVtXlwMKd5KGR1Sp3Nenlm5svD+cEEAAhCAwPwEcp3L8MDggZnogelCwJx5+21XPgxu\/seXO0AAAhAYLwEEzAhtn6vR60wpT4iERFcChtBVnYX4HAIQgEA4gVznstF5YPxTRau2Z\/aHRK5Grxv2EjDPX7rkXl+2LHkIScIJAVNnIT6HAAQgEE4g17lsVALGP9HTzs64fPmy27ZtW+VIyNXodcPePDCvLVvmNn7wg3XFW\/v8E++9505cvoyAaY0oN4IABCDAKqQsxoC8Lxs2bHB79uxxOl+i\/PdyJxEwCJgsBj6dgAAERk0g17ls9B6Ys2fPur179+KB8QjggRn1u47OQwACmRFAwGRiUP+o82PHjrlDhw5N7NmkbbxDNw8aKjIL5XQVQkqdPDxUO9FuCEAAAlUEypsfNjl2YEhER+WBUcjonnvuKc6POHXqlDt69Ghhq7ocmLJBdS7JU089NSQ7N2qrCRh96UNr1jT67jyFtfvvE++8k3z10zxt5rsQgAAE+kZg+\/btbseOHTc1K+TcpL71ZVp7RiNg1q9f7w4ePOj8kJG8MTt37nRHjhxxx48fv4mTeWDKZ02MxQPjCxgJCwmMmIIGATOkVwdthQAE+kqg7IHZuHFjIWgQMH21WE275hEwuRm9zoRVHpgTly65T7z\/PgKmDh6fQwACEOgZAXJgemaQWZpTFUJavXr10qqk8j1zNXodOwRMHSE+hwAEIDAcArnOZaMJIdlQU97L3XffXfyVjeyqH8BpAkb7wii5N8ZlIaTUycMx+sI9IQABCPSFAAKmL5ZI2I5cjV6HEAFTR4jPIQABCAyHQK5z2eg8ME2GXK5Gr2NQJWC0xb\/2h8EDU0ePzyEAAQj0i0CucxkCZso4y9XodY+WhXJUzlYdpRAwj1y96h69erUIUdkRBhJTry9fHi1sVceCzyEAAQgMnUCucxkCBgFzEwFfwJjHxQTM5tWr3UsrVkR5nqsEjFY\/6dq8uBilTm4KAQhAIHcCCJjcLVzRv1yNXmfKPgkYCSddKQ+VrOPD5xCAAASGRCDXuQwPDB6Y3nlg\/NDVj95664aQ0pBeGrQVAhCAQB8IIGD6YIXEbcjV6HUYqzwwEhJFKCdBCKksYPy\/17WdzyEAAQhA4EYCuc5leGDwwAR5YBAwvBIhAAEIDJMAAmaYdpur1bkavQ7KNA\/MVz7wAffMrbfW3WKmzy2JFw\/MTPj4EgQgAIFKArnOZXhg8MA08sCkFDDad8aSeGMeIsk7DwIQgEDOBBAwOVt3Qt9yNXqdKX1PiC2jthBSVwIm5gZ6dTz4HAIQgMCQCeQ6l+GBwQNzE4EQAaMN5r547Zr7+qpVrW0y98Q77ziFryyE5HtgEDBDfn3SdghAoEsCCJgu6XdUd65Gr8PpCxitOtJOuBbKMQ+MlWlzVRICps4yfA4BCECgOYFc5zI8MHhgpnpg6gRMmyElBEzzFxPfgAAEIFBHAAFTRyjDz3M1ep2pmnhgYgkY23nX34lXZyRxQQACEIBAMwK5zmV4YPDANPLA\/OGqVUXei3lLYgqYO69fdycuXy7aRw5MsxcWpSEAAQgYAQTMCMdCrkavM+U0D0xZwNjf6+4Z8rkfQpJg8QVMm7k2IW2hDAQgAIFcCOQ6l+GBwQMzCA8MAiaXVyn9gAAEUhNAwKQm3oP6cjV6HdoQD8yJS5fcJ95\/3+GBqaPJ5xCAAAS6JZDrXIYHBg\/MTB4YEzA6VkB5MG1chJDaoMg9IAABCNxIAAEzwhGRq9HrTNnEA9OmgDFRpPZNyoHRRndqn61SqusLn0MAAhAYO4Fc5zI8MHhgbiLge0LkXdHyZVsNZCGjGB6YaQLGVjtZ21iVNPZXMv2HAARCCSBgQkllVC5Xo9eZqC8CRscVqC26TMCYyEHA1FmRzyEAAQj8hECucxkeGDwwlR6Yu65fL5J0yx4YCxnF8sDc+eMfO52BJIEyTcCwKolXMwQgAIEwAgiYME5ZlcrV6HVGktcjVMC8dMstbvPiYt0tgz6XKDIBI4EiITPJA9PmBnpBjaMQBCAAgYESyHUuwwODBybYA6NcmJdWrCi8MtriXwIDATPQNxrNhgAERkMAATMaU\/9nR3M1ep0pJ3lgUggYtU2hq0keGBNOeGDqrMjnEIAABH5CINe5DA8MHpheeWAQMLxyIQABCLRLAAHTLs9B3C1Xo9fBH4IHps0dgOt48DkEIACBIRPIdS4bnQdm9+7d7uGHHy7G4pUrV9y+ffvcqVOnKsdmrkavexCbCBiFlbRiSPkwWjVkYaa6Oqo+VxLvJA+MCZYfvfVWUQYBMwthvgMBCIyRQK5z2agEzJYtW9yuXbvcs88+6w4dOuQOHDjgNmzY4Pbs2ePOnTt307jO1eh1D3BZwKi8\/q0qB8YEjMSLNrubR1iUBYxyYR69erWoV8u3v75qlUPA1FmPzyEAAQjcSCDXuWxUAkaCZe3atW7btm1B4ztXo9d1PkTAmJAoC5h5jhZoImDmqaeu\/3wOAQhAICcCuc5loxEw69evdwcPHnRnz551e\/fuDRqbZvQnn3zSnTlzZuk7Fy5ccPov18sXEvKovL5sWeGB0ZJpXdr3RQJG4kWXQkg6o0hl5hEWWmGkumwVkv6v++oqe2DmqSdXu9EvCEAAAiKwbt264j+79GelSygCcfr06WwgjU7A\/NM\/\/ZP75Cc\/6RYWFoJzYMrWlqB56qmnshkE5Y5MEjASDdrgbpqAmWdfGARMtkOKjkEAAgkJbN++3e3YseOmGhEwCY3QZlXmgVlcXFxK3D169KhbvXp1bQ7M\/v37b\/C4jMUDo11xJVrMAzNNwNgJ1vMKGG2UJ6+L9nlR\/b4H5pu33lpsoGceGZXhggAEIACBGwmUPTAbN24sBA0CZqAjpSqEpKTenTt3uiNHjrjjx4\/f1LNc44Z1JpQH5rXly4tVRXUCRquPPrRmjTMBYzkxdXVUfS5xgoCZhRzfgQAE9C76wrvvuk++915UGOWjU\/QjS\/\/pB1fMS6H6Wa9c57LRhJBkeHlcLl68uJQDIwGzdetW9\/jjj1cupc7V6HUPQaiAkbdFeSq+gNG99fdZLgkYCSatPCp7YCRstApJZSSS5BVq6wymWdrKdyAAgf4QkHh5\/tKl4r2gH1+xLgkV38usH3nFSsnly5dyAmPVrfffrFeuc9moBIz2gNm8efMNISQNiEmrknI1et1DUCdgvrKwUAgJX8AogdfCPQiYOsJ8DgEItElAHmC9f+bxUoS0x\/ID7ceTHUAbu96Qtk0rk+tcNioBIwP7G9mdP39+Yv6LyuZq9LqHoW8Cxn5V4YGpsxyfQ2CcBOydFTsvzhcw8vroh9w8e1+lslauc9noBEyTAZOr0esYhAoYhXv0q0ceF98Do18jtsS6ri7\/83IISeEpuWgRME0oUhYC4yJgQkIHwCrUHPPyBYxt3pmi3nn7lOtchoCZMjL6YHR\/Fc68gzj0+2UBo+8pzmurkCyElFrAPLNyZbHbr7\/\/jF5ech\/LOzOLaAplQjkIQKCfBFIKCV\/ApApbtUG9D3NZG\/0o3wMBEyhg7v7ud4ss83kSqWYxoDaMS71p2zQBo5fF5xcXC9epCRh5XJ64cqVI6NXVpgdG+87oUpJclYCx1U9DcOPOYn++AwEITCeQUkj4AkZeZ70P+57\/InoImBE+Rb7Rn\/qbv+lk5YsEzDx7q8xitioBU2Tfr1hRPLCTBIwEnjwisQWMhJO9OEzApBZ5s3DlOxCAQPsEUgoJX8CkyrtpgxgCpg2KA7tHWcCYdyFVNyy226WAkWhRaEYCxjwudQJm1phwVQ5MlQfGFzCWe5OaUaoxQD0QgMB0AuWVQTF5+XXZ+yq1V36W\/iFgZqE28O\/4Rv9ff\/3XxUSe0l3YdwGjsI1yYyyEJHPbOUazJNPJ22T31GoC3atOwOiFonKpbTPwoU3zIZANgZSeEF\/A2PsKAdPdUCIHZgr7soBR0Vn3OJnFxCZgUk\/OtiOuwjTTPDCTBIzEh7wkTS9fwOje6r8EjPJf9H\/9m5J4\/eRhEzCpbdO0b5SHAATiEEgpJEzA2EKGWd91cUhMvisemNTEe1Df2AWMiYdyCEkPrYSEL2C0C6a\/i25MAWP1Skz6AmbW3JseDDWaAAEIzEDAfuSlEhJlATNruHyGrs71FQTMXPiG+eWuBYwtD+zCAyMBojNFbItsPwcmpoDRvZWYWyzZLnlg9G\/KefEFTPkE61lCV8McnbQaAhBIuQeMaOv943uDETDdjkFCSAEhpANf\/rL7829\/uyiZMoRkAiZ1vZacVidgJDb0QOsh1v\/b8MD4AsYOZbMQkgkYq1e2KCf+zuL56fYRpHYIQGBWAl0LmKF4ffHAzDrCBvw9M7ovYFIO2KEKmFn3ZFEsu6mAKefNDCGhbsCPBE2HQK8I2Dsy1XvZPDD2YyrlD9p5wCNg5qE30O8iYG4OIWnVkTwuyoEp\/1\/CxUJNswiJpgJGLy15YEz0KHwU+yyUgQ5lmg2BLAl0JWC+s2JFksMj2zIaAqYtkgO6DwLmJwJGl14UFiIKETDfVL6KtztvndltZVHZA2P1K+5cDiFZO\/QdCSeVRcDUkeZzCORDQM+9vCKpPCHmgUHA9GMMkQMzxQ4ImBsFjLwqlvNS5YHxPSG27b+8MjqMcdqlfVzszKdJAkYCalr9X7x2belgyX48WrQCAhCITcB24k4tYPRjaSjHCMgGeGBij8Qe3r9rAWO\/LoQm1QOquspJvOaBCREw5gnRyimFm0La7ef6TBMw0zxAdqij\/t\/kSrX8skmbKAsBCIQRSHkOklpkHhhbWLB5cTGsoR2XQsB0bIAuqjejH\/uN33C\/d\/Jk0YRUyWKqyxcwKeutEjB1IRw92H4oR1v76+USsnNxnYCZtA+NH8pS\/ovEi+4Veomvzm8KaWPoPSkHAQikI9CVgLEeImDS2bqqJkJIU\/hXCZiU6\/77IGCEZ9IyZkui9bf\/Nw+MHQEQ8oDPKmCs3lltkvrl1+2jTu0QyI+AeURC3jNt9N7qs\/fiUHLu8MC0Yf2B3aOPAkYPkK6YD47vgZkmYBQe8pcxS0goF8U\/vyiknVUCpvCo\/PQYgUmHSfo7AatM0wsB05QY5SHQLwIImDB7IGDCOGVVqo8CxrbOD8ktmdUYJkom7YTrbyQ3ScAoNKOclZDl1AiYWS3F9yAwbgIpT6IWaRNMTd5vfbAQAqYPVkjcBgTMjYcpVu2EWyVgJEgkfkI3tEPAJB7YVAeBTAikPIkaAdO\/QUMOzBSbTBMwyvXQ8t+QEMmsZq\/KgVF4R+IgZlJvnQfGP4uoLGBsSbTaGLrCxxcwCkNp9ZItU\/RPw9afVfbzi4tLK6XEaFZvFCGkWUcm34NAPwh0JWD0ngv9gdYHUnhg+mCFxG2oEjA2KcuVqMkzppCwPQ7UbasnlYBRP\/WQWh6KvwqpLGBsEzqJD31HAkRXaHLtNAFjZxvpnvqzL2C00kn1IWASPxhUB4GeEND7MOUO3BZCQsD0YwDggQnwwHxnxw7363\/7t0VJEzCWi5JKwJgYkMfDFzQxhpFt6R8qYExIqI3yvFiicSibJgJGQkorDtRGJe6qPgRMjFHAPSHQfwK24CAk166N3vgCJtTD3Ea9894DD8y8BAf4fTP6NAET6mWYpfu+B6YsYGLWWxYw1nZ5XrQDr3lcbBWSCRgJFokREzChwqJOwGgnXzvtuixgTMzNyte8aLN8n+9AAALdEuhCwFiOHwKmW9urdjwwc3pgYgqJPgsY20ZbLxAJDHlrJGDuvH69EDnyjoRuEIeA6f5FQAsgMEQCloOX0gNjAibmu79tW+CBaZvoAO4X4oGJqcKHKGAkXPRSkZcmNMF5VgGjIdREKJWHHEm8A3gIaSIEphAwb7HlysWGJU8wAiY25fD744GZ0QNjybQpBczry5cXq290xay3KoSkVUF2QKMl09pOvL4Hxt90LnQYziJgjL\/CV7PuwomACbUQ5SDQTwIImDC74IEJ45RVqWkemLEIGIkLCZRpAkZGnyeZNlTASKzokmBBwGT1qNEZCDQmoHeO3gMxf8yVG2WrT4v30OrVxQqoIVwImCFYqUEbt2zZ4nbu3OmOHDnijh8\/XvnNEAETcy8AP4Skh1QPS2oPTIiAsROgQ5N2y7ARMA0GLkUhAIGCgAmYlEICAdOvwTfKENL69evdwYMH3W233eYOHz7cSMCYYLHlzDEFjP+wjE3A6DGxvRZsFZLCU7bng3lgmuTalB89Qkj9ehkO7SpBAAAgAElEQVTRGgg0IdC1gAndJqJJn2KVxQMTi2wH9929e7fbvHlzUXOIB+b\/\/fqvu19+6aUiYdTO90klYPwleyk8ML5bVud9aJlxOYSkUI4+0wNs++GIZVseGDsQcpqAsXoRMB08QFQJgR4QQMCEGwEBE86q1yU3bdrkHnnkEffiiy8WIiZUwNz93e8W\/UotYLTviR1Z4AuYWJ6fEAFjJz+nEDC2kkneKN8DYwJmHg54YHr9qNI4CEwlgIAJHyAImHBWvS554MCBon2nT58OzoG59ju\/4yRgLly44F782MeKE5b\/zz\/\/85KgCV0u3BRMeddHTeDaY0XXPBP3tHZMEzDqt0I3VQJmnuXM5RwY3wMjttZvBEzTEUR5CORLwN4bXeXA9DmEtG7dOqf\/7NKf9+3b53bt2lXMfblco8qBUeLugw8+6B577DF37733BguYDx065Bafe66w+aUHH\/zJHid\/9EdZCxg718gPIfkCRmElrQYyT0hbAkYvBXlGfM+TL9wsZGT5QfMIOTwwubzG6McYCZiASSkk\/LzEWUPmKWy1fft2t2PHjpuqQsCkoB+pjqNHj7pXXnnFHTp0yDVZhaQcGAshyQOjSfR\/\/uu\/jkbAqKMKX\/VBwJhgsRfJPEsoETCRHjRuC4EEBBAwkyGXPTAbN24sBA0CJsHAjFGFcl\/279\/vFhYWbrr9sWPHClFTvixu6AsYiZdv3nrr0nLmeZJI6\/o5LYQUq14\/rqzcG3lgdPn9lkfEPDAmJFJ5YEzA2BLzeX59IWDqRiCfQ6C\/BBAw4bYhByac1SBKzuqB8T0RhWdijp1g60CFCBgJDp0\/1NaGSrMKmHk4+DkwdSEkEzD6jgTWPGegIGDqRiCfQ6C\/BBAw4bZBwISzGkTJNgSM74mI0WlfwGjilkiwgxJtPxR5SFRunlCK3\/ahCJg2eCNg2qDIPSDQDQF7983jhW3a8jY8zk3rbKM8AqYNigO7RzmEZPuhVOWCxOiaEmR1ac8VhXCqNnRrIxdkmoB59OrVpf1v\/NCZeVys\/rY9MBa6Km\/gN0\/SbtlGCJgYo5Z7QiANARMwKZNpETBpbBtay6hWIYVCsXImYP5j61a36uWXb9rQLbYHZpKAkVjQ2USa3G0VUCwPDAKm6aihPAQgkIIAAiacMh6YcFbZlOyDgJFQUazX98BUCZi2knrLISRfwMjzZDsQm8fFkmnnqb8qB8Y8MFrOHesUbjww2TyqdGSEBBAw4UZHwISzyqakL2De\/Id\/KPrln8rsb6mvSdhyUdpKqJV3pUrASCxonxR\/H5Z5BMSkEJLEjO2AW96B2OqLIWC+8O67TsJJFwImm8eJjkCgVQIImHCcCJhwVtmUrBMwmsQlXGzljCbdtoSEIIYIGDvUcJ4clDYEzDy5KWUPDAImm0eIjkAgGgH78UQOTD1iBEw9o+xKNBEwMZK7mgiYefZh8Q3nb89tHhh9Xj6F24Sa\/QpCwGQ3\/OkQBHpNoIsQcBuLFrqAioDpgnrHdVYJGJvMtZzZ98C0cSpzubtlAaPPzctjISTLSclZwCgkZ\/1sK1lZLLt4AXY8pKkeAtkQ6OL5RcD0a\/iwCmmKPaYJGA1kTazyQMiF6QuYtvYl8AWM6pJIUX36sy9gFD7Spm5tuFJn9cDMIyz8EJL6YK5hmcYOajMB0+bBbV28APv1+NMaCAyXQBfPLwKmX+MFAdNAwCihVsJBnhcNZIVN5BHRpKtcFF0Ku7Q1yU4TMJZ7o4k9hoCRCLPEZPXLBIoJCT\/XR+XmSVxGwPTrpUBrIDAEAgiYcCsRQgpnlU3JsgemLGA0qUvImIDRpC5BM483wodnAkaiSXWbB0b1yBPz+cXFQjiZgGnD8+Nvz10lYCxpeJ6cl\/IAQcBk88jQEQgkI4CACUeNgAlnlU1JM\/p\/+dKX3JnTpwsR4XtgfAEjz4QmdQkLW3I8L4hCnPw0XDRNwJgnCAHTjHgXL8BmLaQ0BCAwiYB+PJonOhUlQkipSIfVQwgpIIQUKmAkaDQp2jlFYSaYXEoCRmLok++9N9UDY0KqjdBVHzwwtrJJZEyUmeenDZFmxBEw845Qvg+B7gjYWXHaDyvVZQKmze0yUrQdD0wKyj2rI9QDI+GgVUkSEkqmtcTeebvTVMC0Ebrqq4CxJGkEzLyjiu9DIA8CCJhwOyJgwlllU9IXMM9\/\/\/tFv6pCSCZgbNfY5y9dKpJ557lsl1\/fA6P72bECyrXx6zUBpfLzXH0VMHhg5rEq34VAfgQQMOE2RcCEs8qmZJWAmSYgLIQj8aJys146fdq20ld+i4WQTMDoTCI9vCZg5JWQaGoj98YXMHdev154lnSZdydFEu+0EFIbS8XNLoSQZh2hfA8C3RNAwITbAAETziqbktMEjCbZ4jToy5dv8ITMs5zYwNmBivp7WcD4HqCygGkj9yZUwLQRrrL+llchVQkYCyEhYLJ5vOgIBOYigIAJx4eACWeVTcmygCkvYzYB02YSreBVCRj9e3kVlC9gnrhypfhcbZnnmiRgzLtkQiK1gImx4gAPzDwjhe9CoFsCehfpIom33g4ImHpG2ZUIFTBtLmOuEjASNLYXzKRl3JYdr3DSPJeFjXQfP4TUtYAxLhKRbV0ImLZIch8IpCfQpYBpcx+sFOQQMCko96yOOgFjG8mZB6atFTJlD8wkAePvBGybzs2bPCyBoHv0RcC0GTIqD68uTrPt2RCnORAYLAEETLjpEDDhrLIpGSpgbGfctibbWQSMeSjkNZn10hJwSx6uEzBt7Dlj7bSwlR1I6efAtMW0igkCZtaRwvcg0D0B26l83rB5k56YpxsPTBNq8cqykd0UtiZg1j\/wgHv2zTcnbuWfSsBYU\/Xw2GnYbe05o3uXk2n9v5tgsQc4poAptyPW8EfAxCLLfSEQnwACJpwxHphwVtmUnCRgtNpHk6yFkCzs0pa3YJIHpkrAtLmVNgImm6FLRyCQPQEETLiJETDhrLIp6QuY\/\/bWW0W\/FGKRx0XJtMp+1xlIllg6bwKtgfMFjO3uq\/p0aaXRMytXLnlgYgoYvx1lD0xb+T6+58dCSHhgsnmE6AgEohFAwISjRcCEs8qmJALmx8Vp17pMwFjYBQGTzTCnIxAYJAEETLjZEDDhrLIpWRYwry9bVuyA24UHRl4J1e97YNr2\/JQ9H74HxgSLcm70X5t7L5STeMt\/jzWgyIGJRZb7QiA+AQRMOGMETDirbEqGChh1WGcXtTWpV4WQfAGjowTkGZGAkahpq94QARPDuAiYGFS5JwTyJqB3YBu7jzehZIsY2tzIs0n9s5ZFwMxKbsDf64uA0dlI8nqYByZ3AWMCznJiYg0hPDCxyHJfCMQnIAHTxvlvTVqKgGlCK35ZllFPYTxJwMjbokueD9taP6YHxgRM4elZscKZgGnb89MXDwwCJv6DTw0QGDoBBEy4BfHAhLPKpmSIgLHTmREws5u9HEJCwMzOkm9CYCwEEDDhlkbAhLPqbclNmza5\/fv3u4WFhaKN58+fd3v27HHnzp2rbPM0AWMHJ5qAkSuzrR0hyzkwvgfGXKZavq2rzXqrli9bPW2uOirDniRg2hSFVQYmhNTbR5WGQaCWAAKmFtFSAQRMOKtelly\/fr07ePCgO3v2rNu7d6+zv1++fNlt27ZtqoD5r3ff7bT7ra1CUm6GJY9ZCKnNraX7KGDa2qSvCnTVqiMJJwRMLx8lGgWBXhBAwISbAQETzmowJQ8cOOA2bNgw0QtjRu9SwGj\/FTujSMKp7IFpUzhN88AgYAYzrGkoBEZBAAETbmYETDirwZRsImD+4OMfL5Ytf+Pf\/q34fyoPTBcCxl\/9YyGkFALG97jggRnMY0RDIdAJAQTMZOzr1q1z+s8u\/Xnfvn1u165d7vTp053YK0alo12FZPkwr7zyShFSqrp8D8zFAwfcex\/5iLvzS18q\/q\/DHZXzEmNZXXkLf\/1d9fgeGMu9ieGB6YOAUX6KQnbyOMW6yIGJRZb7QiA+Af3IafP9F9LiGIfZhtTbtMz27dvdjh07bvoaAqYpyR6Wt\/wXNS0kiVchJHlgJFx+7+TJokeWPBtjQE8SMKrXHlgTMG1uqFSVi2JHCbR1zlPVcLB6Y+e8lOtGwPTw4aRJEAgkgIAJ98Bs3LixEDQImMDB1ddioeJF7fc9MBIK8kycuHy56JoJCZsE7aygNvqNgGmDYv09EDD1jCgBgb4SkIBp8wdcSD9j\/GANqXfeMuTAzEuwB98PWXnkNzNEwEhs6LJzidro5jQBYw+seWDaFE54YNqwHveAAARSEEDAhFNGwISz6m3Jo0ePutWrV08NGzUVMDE6GyJgbPl2CgHT5nlLVbwIIcUYRdwTAnkTQMCE2xcBE86qlyXLm9hZI69cuVJkZ586deqmdod4YGJ0NkTAxHBlVnlgdAaTlnG3tUkfAibGiOGeEBgfAQRMuM0RMOGssikpo\/\/ZY4+59Q88UEzgWjptSa2xs99t+bI8LLYKSWAthGQCps3lzVUCJoUx8cCkoEwdEMiLAAIm3J4ImHBW2ZTss4CRqLnz+vVCVLV1IWDaIsl9IACB2AQQMOGEETDhrLIp6QsYeUJeX758yQMTO\/vdP4NIQsVWP7WZ81I2FAImm6FLRyCQPQEETLiJETDhrLIpiYBJY0pCSGk4UwsEciFgeYKxf0iWecXIPUxhEwRMCso9qwMBk8YgXQkYJSjrhdRmHlEaYtQCgXETQMA0sz8CphmvLEqXBYzyTSy0E1v5jzGEZLsbpxo8JmAUlvvitWvu66tWtbqfT10\/JNy0wuubt96atN66dvE5BPpOwARMzJB6FQPzwGhn8jb3\/orNGwETm3AP7z9NwMR+cLoUMKm39DcPTFcCRivKHr16del4iFRDMcY5WqnaTj0Q6JIAAqYZfQRMM15ZlO6LgBFMW74dUzh1FcrpWsBIOMkboytlOMk2I0wt3LJ4OOnEqAkgYJqZHwHTjFcWpWX0P9+6tTiB2oSDvz9Lm0uYy8B8D4wvYGK6LscqYORxUihHV0y+ZRubgEktnLJ4OOnEqAkgYJqZHwHTjFcWpREwaczYtQdGAkaXRIzCScpJiX0pfm5eNb2MUwqn2H3j\/hCITQAB04wwAqYZryxK+wLGJhg7RDH2hGMeGIU0\/KMFYtY7Zg\/Ma8uXu2dWrlzabyf2AFYSuHJgLP9GIkptiH3FPBIidtu5PwSMAAKm2VhAwDTjlUVpBEwaM3btgZE3RHkoWoVkOxzH7LmEiw7ItDOmVL9WQd11\/XrMaov6UidoR+0QNx8tAQRMM9MjYJrxyqJ0XwSMYPoemVhwx+qBEc\/YZ1v5NpOAEWu9hGMvxy\/XK5G0eXEx1hDivhBIQgAB0wwzAqYZryxKTxMwsVerlAULAqb9IWX7wOjOqYWECZiYq8rKxCScEDDtjyPumJ4AAqYZcwRMM15ZlK4SMLZyBAHTnom7DiEhYNqzJXeCQAoCCJhmlBEwzXhlUbpKwJj7X8m0Ma8qD4xyJWLW23UIKWUYR7bzPTCpPSG270zqevHAxHxquXcqAvauSvn8qG\/sxJvKwmH1\/MyaNWv+I6zo+EpVCRgpf12xt5EuCxhbcouAaW8c9kHAxFxVRgipvbHCnfpFoGsBE9sD3zZtPDBtEx3A\/WT0\/\/7pT7u1e\/cm36dDAsb3uEjAaOVKzATMrkI5Vu8YPTAImAG8CGhi7wggYJqZBAHTjFcWpX0Bk1pxlwVMCs\/PmAVMaiHRxdEFJPFm8VqiE9p08r33ij2bUj63fggp9Xwwr9ERMPMSHOD3+yRgUuBDwCxLgXkpjq7KUr4IETBJzEslCQggYJpBRsA045VF6bEJGMvsT324YB9CSKmFhDwwsZOyyYHJ4jVEJyoIIGCaDQsETDNeWZRGwKQxIwImDWc8MGk4U0t8AgiYZowRMM14ZVFaRv\/mz\/+8+9ATTyR19QteOQcmBdCxemC68ITggUkxoqkjVwIImGaWRcA045VFaRMw7\/zJn0Tdf6UKFgIm\/hCyZdRdCZjU5xLhgYk\/pqghDQEETDPOCJhmvLIo3bWAST3BmQcm9XJmexml3M5fAxQBk8VjSidGSAAB08zoCJhmvLIojYBJY8auBUxqoWi7eXZRr1jH3AwxzYihlrETQMA0GwEImGa8siiNgEljxrEKmNSrvVIdg5Fm1FDLmAkgYJpZHwHTjFcWpWX0v1i92r39\/PPJf7UqByb1L\/SxhpBSczYPTOpQHQImi9cSneh4I7shejERMJk8NgcOHHD3339\/0ZuTJ0+6vXv3TuyZCZj\/\/T\/+R9Qt\/KsagICJP+AsB6YLT4jqTi1gHrl6tcj7IYQUf2xRQ1wC9ux2sRMvAiaubZvcfVSHOW7ZssXt3LnTHTlypGBkfz5+\/Hgls64FTOqJdawemNRCwjwwqZOWETBNXo2U7TMBBEwz6+CBacarl6XlfdmwYYPbs2ePO3funDt69Ki7ePHiRC\/MWAVM6om1q4PZ7CXYhYBRn7++apWTSE11IWBSkaae2ATs2U25g7b6NNQwLAIm9ohMcH8JFl3btm0r\/l\/+e7kJMvrzly65M6dPdxJCSj2xmgdmbAImdX8lJB69etVtXr3avbRiRYKR\/5MqEDDJUFNRZAIImGaAETDNePWydNnjIo\/M2rVrlwTNJAHz8htvuD\/4+MeXPr5w4YLTfzEv5cAgYGIS\/s99YLoQEhIwqeP3CJi444m7pyOAgJnOet26dU7\/2aU\/79u3z+3atcudPn06naEi1zSqHJhZBMyfb91amODqffctmeLJJ590Tz31VFTTKMTw+vLlxYF\/Ka8u603pjRBTeZzU35RhHKv3C+++W4SQUl5dCRi53TWOU\/dX9WqFWWr7nrh0qfjxkXI8axxLFKuvKftr9X5lYSHpu6orAdPVO6Ppe2L79u1ux44dN30NAdOUZI\/KzxJCOnz4sNu\/f\/8NHpcUHpgeYaMpmRAwAfP6smXuteXLnUJnKS5N6Lo2Ly6mqG6pji68mKpc9aYOS3a9l1Jqb2JXAibpAJ6jsrIHZuPGjYWgQcDMAbXrr5ZDRiFJvBIwuRm9aztQfzcETMCodomYO3\/848JLEFvImICRaNIVuz6ji4CJP85MSKQOwyJgmtmWHJhmvHpZepZl1AiYXpqSRs1AwJKHFc7xBYyF0WKFeM68\/XZR3yfef78IbyBgZjBezVfMA5M6b86ERGqPEwKm2RhCwDTj1dvSTTeyQ8D01pQ0rCEBX8Doq4rnK0dEwkL\/\/86KFcX\/287dKAuYVEvI5YFJKZi69PyYgEndXwRMw4ewo+IImI7Ad1ltrkbvkil1d0fAJhu1QF4YCRj7vwmZGL\/gJWBMMFl9KSZaBEz8sdbVXkp4YJrZNte5bFSrkJqZ3Llcjd6UA+XzIBAiYCQs2vaQSMBILPnCyVYHSdC07fHxPSEphFJ5dHSRe9O1ByY1ZwRMs3dSrnMZAmbKOMjV6M2GPqVzIeALmHKfyh6ZNnMaJgkYha7arKdKSKSeWLs+jiP1waRdnSeGgGn2Vsp1LkPAIGCaPQmUHiyBaQLGOuWHeO66fr2V\/UyqBIzVEyNk1aUHpmsBo76n3F7fxlRXwillXwf74Lt8owkIGATMkJ9L2t6AQBMBYzkxbXhIfAFTbq7q0RVj47cucmDGKmBSCydLSEfAhL0A8MCEccqqVK5Gz8pIdCaYQIiAKXtiJCy+eeutc+2yOk3AmCemDaFECOlygSDlpnL+mEopJhAwwY99UTDXuQwPDB6YZk8CpQdLwBI9m3TAPDHzTIrTBIy1JTcBkzr3xrdtDJaTxowJidTCCQHT5ClGwDSjlUnpXFVrJuahGw0JzCJg2hAYIQJGE768MfN6e3wkCiGlzs2wENLYBIyddZXqHKauzvVq+Mj1pniucxkeGDwwvXnIaEhcAvMKGC13nuVw0RABEyOUNFYBk1I8+cdTpDiWwp4QBEyzdwUCphmvLErnavQsjEMnGhOYR8CYwJgllBSSA9OGp6cMZIwCxuyUKh\/FhITt5ZPqmAgETLPHP9e5DA8MHphmTwKlB0tgHgFjnZ7l0L4QDwwCZr5h5Z+F9OjVq24WO83SAhMSOkfriXfeibqvj98+BEwzayFgmvHKonSuRs\/COHSiMYE2BMwsCaJNBIwdNPmVhYWZwlUGxXJRxpYDIw\/ZF95910nEpLosZKVTx7U5YczdlZfse\/16cZq6+stVTyDXuQwPDB6Y+tFPiSwItCFgtKxa4kCb3IVemtSaXrMIJb+OMQsYCQgtb051+blR4i4B9cn33otefYy9g6I3uqMKEDAdge+y2lyN3iVT6u6OgE3q87ag6S9siZ2mImbeHXqtr2pryl\/pXa9CmiVHad7xwPf7TyDXuQwPDB6Y\/j99tLAVAm0JmKbiYhbPj0I\/CiPNcyl01ZWASR26MsYImHlGTL7fRcDka9uJPcvV6CM0JV12rgj9aFKf94otYGwlzbzt1PcRMG1Q5B5DJ5DrXIYHBg\/M0J9N2h9IoC0B0\/RXflMPjB1fcGeDPJsqBF+8ds2p7i5CSHhgAgclxZIQQMAkwdyvSnI1er8o05pUBNoQMPJoKMHW9v0IaXtTATNvAq+1qYultl0lDxNCChmJ4y2T61yGBwYPzHif6pH1fF4BY8Kl6W68TQVMUw\/PJDMiYEY2wOnuRAIImBEOjlyNPkJT0uUWcmBm3RytTsBYzovCLq8tX+6eWbmykYcHAeOKUNmJy5eTnkTNQzUcArnOZXhg8MAM5ymkpXMT0Pb6TS8TGLEEjLWnaXJwXT+6OLG4q+XbCJi60TDuzxEwI7R\/rkYfoSnp8k8JzCJgtNOqvCOznjQcGrpqK\/fFjK3N3LS9fapzgVQvAoZHrY8Ecp3L8MDggenj80abIhFoImDaOiG6TsDM6+GZhAoBE2kQcdvBEUDADM5k8zc4V6PPT4Y7DJXALAJm1tCRMaoTMLZsWuWbJghPswMCZqijlHa3TSDXuQwPDB6Ytp8V7tdjAk0EjIWM5k2qnSRgzPPSdu4LIaRlPR6BNK0LAgiYLqh3XGeuRu8YK9V3SKBKwJR3vlW+i84uaisnpU7AtFVPGSsemA4HGlX3ikCucxkeGDwwvXrQaExcAtMEjAkZeV40+c8bOvJ74tfre14kll5fvrzV0FHZA9PWvjIhlvHFWsrkYVYhhVhnvGUQMCO0fa5GH6Ep6fJPCegsJE2yukxI+PuwyPMSI6QjAVOuL0Y9vqHNA4OAYfiPnUCucxkeGDwwY3+2R9V\/O8xRIqZKUHzyvfcKAdPkqIAQgBIwFpqSh+eu69eLZdmzLs0OqbMLrwQemBDLUCY1AQRMauIR6tu0aZPbv3+\/W1hYKO5+\/vx5t2fPHnfu3LnK2nI1egS03HIgBHwBY4LC\/h\/TI1IWMMp7iX0hYGIT5v5DIZDrXDYaD8z69evdwYMH3dmzZ93evXud\/f3y5ctu27ZtCJihPIm0cy4CJmB0k9eXLSuSdQtPyMqV0XJRVJcvYGIKJR8OAmauocKXMyKAgMnImNaVAwcOuA0bNkz0wuRq9AxNSZcCCZy4dMnd+dMcGAkYiReFktoOGZWbI+Gk+jYvLga2dP5iJmDaTEaua5UfQkqZe9OFWKtjwef9IZDrXDYaD0zVUAoVME8++aQ7c+bM0i0uXLjg9B8XBIZGQAJG13dWrCgERcwcFJ+N6lWdX1+1KhkyExMImGTIqagnBNatW+f0n1368759+9yuXbvc6dOne9LK+ZsxWgFj+TCvvPJKEVKquky1lj+ToHnqqafmp88dIJCYgH6pa2JPJVwSd++G6hAwXdKn7i4JbN++3e3YseOmJiBgurRKS3Vb\/otuF5LEq8Rf3+OCB6YlQ3AbCEQkgICJCJdb95pA2QOzcePGQtAgYHpttv9s3JYtWwpjrVy5svjHY8eOuUOHDi0l79aJF32ea9xwICakmRCYi4AJmFg7\/VY1jhyYuUzGlyMRyHUuG1UIKWTlkT9+cjV6pGeE20KgdwS0+qkrAZMy94Yk3t4NvV41KNe5bFQC5ujRo2716tVTw0YImF49dzQGAnMRQMDMhY8vZ0IAATNwQ5Y3sbPuXLlypcjOPnXq1E09zNXoAzclzYdAMAEETDAqCmZMINe5bFQemKbjM1ejN+VAeQgMlUCXAiZl6IoQ0lBHaJp25zqXIWCmjJ9cjZ7mkaEWCHRPQAIm1c6\/6q2fxIuA6d7+tOAnBHKdyxAwCBiecQhkS0A7AGvPm1Qb6CFgsh1Kg+4YAmbQ5put8bkafTYafAsCwyOAgBmezWhx+wRyncvwwOCBaf9p4Y4Q6AkBCRid85Ti9OtyCCll6IocmJ4MuJ42AwHTU8PEbFauRo\/JjHtDoE8EdAbTa8uXI2D6ZBTakpxArnMZHhg8MMkfJiqEQCoCCJhUpKmnzwQQMH22TqS25Wr0SLi4LQR6R6BLAaPk4VShK0JIvRt6vWpQrnMZHhg8ML160GgMBNok8MQ777i7rl93mxcX27ztxHv5q5AQMEmQU0kAAQRMAKTciuRq9NzsRH8gMIkAAoaxAQH2gRnlGEDAjNLsdDojAhIwCq9s\/OAHk\/QKD0wSzFTSkECucxkhJEJIDR8FikNgOAQeuXrVffHdd5MLmNeWLXOvL1uWLHRFDsxwxmQXLUXAdEG94zpzNXrHWKkeAskISMA8evWq+9CaNUnqNA8MAiYJbioJJJDrXIYHBg9M4CNAMQgMj4C8LwojdSFgRCtV6AoPzPDGZsoWI2BS0u5JXbkavSd4aQYEohPoSsC8dMst7hPvv59MOCFgog+lQVeQ61yGBwYPzKAfTBoPgWkETMDIE6KwTuzLQkhaQq26U3l+EDCxLTvs+yNghm2\/mVqfq9FngsGXIDBAAqkndgTMAAfJCJqc61yGBwYPzAgeX7o4VgImYDavXl0c6hj7KguYVJ6f1EItNkfu3y4BBEy7PAdxt1yNPk7PszoAAAhESURBVAj4NBICLRCQoHj+0iX39VWrnMI6sS8TMDqJWqufEDCxiXP\/EAK5zmV4YPDAhIx\/ykBgsAR+9NZbxZlEKQWM6tPqp1SeHzwwgx2eSRqOgEmCuV+V5Gr0flGmNRCIS0ACRh4ReWFiX+aBkXA5cflyMuGEgIlt2WHfP9e5DA8MHphhP5m0HgI1BM68\/XaR\/5LiZGgEDMOxjwQQMH20SuQ25Wr0yNi4PQR6ReDEpUvuteXLkwsY5cB8Z8WKJJ4fPDC9GnK9a0yucxkeGDwwvXvYaBAE2iSQ8kRq3wOjelN5fhAwbY6Y\/O6FgMnPprU9ytXotR2nAAQyIpDyQEdfwHzx2jV31\/XrSQ50RMBkNGAjdCXXuQwPDB6YCI8Lt4RAfwik3I3XFzA6SiDVQZIImP6Mtz62BAHTR6tEblOuRo+MjdtDoFcENLkrnPP5xcXoxwn4AkZ\/Vr0p9oJBwPRqyPWuMbnOZXhg8MD07mGjQRBom0CqvWB8AaP8l1T1ImDaHjF53Q8Bk5c93ZYtW9zOnTvdkSNH3PHjxyt7l6vRMzMl3YFALYFUK5HKAibVEm4ETO0QGHWBXOeyUXpg1q9f7w4ePOhuu+02d\/jw4VEKmHXr1rnPfvaz7oUXXnAXLlzI5uGmX8MyZSp7pcqD8QXM\/73rLvcXq1e7u\/\/+76OfSp1SwKSyWeqRnGu\/xBEBk3o0Raxv9+7dbvPmzUUNY\/XA5Dqg6VfEByfCrVPaS\/koEjKxr9eWLSv2nLm6aZP7s8cec+sfeKDIvXl92bKoVStpOEW+TUqbRQVWunmu\/ULApBxFkevatGmTe+SRR9yLL75YiBgEzC53+vTpyNTT3T7XlxD9amcMaUl17OulW24p9n8xmx348pfdL\/\/d38Wu1j2zcmWSE7cZi9FN2XoFudpsdCGkAwcOFINDk3ZoDsyTTz7pzpw50\/qg6vKGcpfu27fP5dY3+tXlqGped672Eolc+0a\/mo\/zrr9hNtu1K68frKMSMErcffDBB91jjz3m7r333loBY0aXeuWCAAQgAAEIDJWAfrTv378\/q5zHbAWMxIrU5sqVK4vxduzYMXfPPfe4V155xR06dChoFZL9ipKQ4YIABCAAAQgMlYAWa+S0YEN2yFbAlAeZcl+kPhcWFm4afxI3EjVcEIAABCAAAQgMg8BoBEzZHCH7wAzDhLQSAhCAAAQgMD4CCJgpG9mNbzjQYwhAAAIQgMAwCIxWwAzDPLQSAhCAAAQgAIEqAggYxgUEIAABCEAAAoMjgIAZnMloMAQgAAEIQAACCJgpY+Do0aPu7rvvLkrktlLJzoPSjsQ5rMAqL5t\/9dVX3bZt27J4wssr6E6ePOn27t2bRd+sE3a8hzZXPHXq1KD7VrXiMSebaTPQ+++\/v7DR+fPn3Z49e9y5c+cGa7NJK1SvXLlSbPY59PHoz2M5vRc14BAwEx47\/4V63333FccO5DCY1V0TL3fccUcWwsxeQCdOnCjEmP1de\/4MfaI3W509e7boiwm1Z599NgvhqfFo9tKfc3jGZKOtW7e6xx9\/fPCTX\/n1qPfiQw89tHQIriZHXbn8WPDfj\/bMDVaZOeckNjds2FCIzNtvv73YSiSH96LZBAEzYXT6D2ZO3gqbAN988023uLjobNIf8kNa1fYcX6y5vVzNbnrJ\/uIv\/mKx6WQOAkaT\/Kc+9anBeyYmPVcXL14c\/A+Dae87f9IfsmdJfSy\/B3N7LyJgKkZy+Vdv+e9Dnuw\/85nPFM1\/++23CzWOgBmWNXPbv8iO9\/jHf\/xH9+lPfzoLAaMJcO3atVl5JXxPWa7vjBz7WOWBycl+CJgpAsbPD5FyzemXRznsMqxpfHprcwyz+GG\/nOLYeq7k0taVS5jWzzlQv3LIE7HJ\/atf\/ap7+eWX3QMPPFB4zHLpm+8NtJDL0L0v1id5BB9++GF37dq1pdBfLu97BAwCJptcCv8X1A9+8IPsfgH7IaTLly8Pvn96sep8MuVP5JLEa0LT7FP++5AnDvvRc+nSpSI8puvgwYMuh7GYa3hWYnr16tU32CuH3B57jhAwIwshWXdz9MBYn3IVL\/4vqqF7KzSxf+1rX3PPPfecO378eDYCpkqg5CLOqt4ZufTN9zA9\/fTTxZgc+pW7vWQfBMyEUeqHjHJK4s1VwOS08qjuxZlDkmF52bv1OUc3dy5JvVXvwVz6pvGXkxiblM+TWx8RMBNmi5yXUU8a3HUTZ18\/z8lNX2ZcTiDPVajl8mLNeUm\/xqYvni2ElEtIIsfk66oQUi4hPzwwNTNyzhvZ5RRCmvRrPpcEw\/JGWzkl8eYUFit7NxcWFop\/ys1e\/kZ2OfUttyXGfl6P9vzKcSzigemrW4F2QQACEIAABCAwkQAChsEBAQhAAAIQgMDgCCBgBmcyGgwBCEAAAhCAAAKGMQABCEAAAhCAwOAIIGAGZzIaDAEIQAACEIAAAoYxAAEIQAACEIDA4AggYAZnMhoMAQhAAAIQgAAChjEAAQhAAAIQgMDgCCBgBmcyGgwBCEAAAhCAAAKGMQABCEAAAhCAwOAIIGAGZzIaDAEIQAACEIAAAoYxAAEIQAACEIDA4AggYAZnMhoMAQhAAAIQgAAChjEAAQhAAAIQgMDgCCBgBmcyGgwBCEAAAhCAAAKGMQABCEAAAhCAwOAIIGAGZzIaDAEIQAACEIAAAoYxAAEIQAACEIDA4AggYAZnMhoMAQhAAAIQgAAChjEAAQhAAAIQgMDgCCBgBmcyGgwBCEAAAhCAAAKGMQABCEAAAhCAwOAIIGAGZzIaDAEIQAACEIAAAoYxAAEIQAACEIDA4Aj8f2HpdE8j8QZiAAAAAElFTkSuQmCC","height":337.04307334109427,"width":560}}
%---
%[output:7926a099]
%   data: {"dataType":"warning","outputData":{"text":"Warning: The file containing block diagram '<a href=\"matlab:open_system ('aru_toolbox')\">aru_toolbox<\/a>' has been changed on disk since it was loaded. You should close it, and Simulink will reload it if necessary."}}
%---
%[output:5e4e4070]
%   data: {"dataType":"warning","outputData":{"text":"Warning: The file containing block diagram '<a href=\"matlab:open_system ('aru_toolbox')\">aru_toolbox<\/a>' has been changed on disk since it was loaded. You should close it, and Simulink will reload it if necessary."}}
%---
%[output:186d884a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: The file containing block diagram '<a href=\"matlab:open_system ('aru_toolbox')\">aru_toolbox<\/a>' has been changed on disk since it was loaded. You should close it, and Simulink will reload it if necessary."}}
%---
%[output:53ed4466]
%   data: {"dataType":"warning","outputData":{"text":"Warning: The file containing block diagram '<a href=\"matlab:open_system ('aru_toolbox')\">aru_toolbox<\/a>' has been changed on disk since it was loaded. You should close it, and Simulink will reload it if necessary."}}
%---
%[output:4bcb810a]
%   data: {"dataType":"warning","outputData":{"text":"Warning: The file containing block diagram '<a href=\"matlab:open_system ('aru_toolbox')\">aru_toolbox<\/a>' has been changed on disk since it was loaded. You should close it, and Simulink will reload it if necessary."}}
%---
%[output:687d07b3]
%   data: {"dataType":"error","outputData":{"errorType":"runtime","text":"Error using <a href=\"matlab:matlab.lang.internal.introspective.errorDocCallback('LSQ_motor_model>AK10lsqPos\/tracklsq', 'C:\\Users\\gfben\\OneDrive\\Documents\\MSc\\Baleka-motor-ID\\LSQ\\LSQ_motor_model.m', 121)\" style=\"font-weight:bold\">LSQ_motor_model>AK10lsqPos\/tracklsq<\/a> (<a href=\"matlab: opentoline('C:\\Users\\gfben\\OneDrive\\Documents\\MSc\\Baleka-motor-ID\\LSQ\\LSQ_motor_model.m',121,0)\">line 121<\/a>)\nSimulation aborted\n\nError in <a href=\"matlab:matlab.lang.internal.introspective.errorDocCallback('lsqnonlin', 'C:\\Program Files\\MATLAB\\R2025a\\toolbox\\shared\\optimlib\\lsqnonlin.m', 221)\" style=\"font-weight:bold\">lsqnonlin<\/a> (<a href=\"matlab: opentoline('C:\\Program Files\\MATLAB\\R2025a\\toolbox\\shared\\optimlib\\lsqnonlin.m',221,0)\">line 221<\/a>)\n            initVals.F = feval(funfcn{3},xCurrent,varargin{:});\n            ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\nError in <a href=\"matlab:matlab.lang.internal.introspective.errorDocCallback('LSQ_motor_model>AK10lsqPos', 'C:\\Users\\gfben\\OneDrive\\Documents\\MSc\\Baleka-motor-ID\\LSQ\\LSQ_motor_model.m', 80)\" style=\"font-weight:bold\">LSQ_motor_model>AK10lsqPos<\/a> (<a href=\"matlab: opentoline('C:\\Users\\gfben\\OneDrive\\Documents\\MSc\\Baleka-motor-ID\\LSQ\\LSQ_motor_model.m',80,0)\">line 80<\/a>)\npid = lsqnonlin(@tracklsq,pid0,[1.0, 0.5, 0.01],[1.7, 1, 0.5], options);\n^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^\nCaused by:\n    Failure in initial objective function evaluation. LSQNONLIN cannot continue."}}
%---
