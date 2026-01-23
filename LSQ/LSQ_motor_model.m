clear;
clc;
% Initial guesses 
hold off %[output:1ef8beac]




qStepSize = 0;
wStart = 0;
wStepSize = 0;
Kp = 0;
Kd = 0;
StepTime = 0;
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
[J, b, Bt, Ct] = AK10lsqPos
%%
function [J, b, Bt, Ct] = AK10lsqPos

%% LSQ for inertia (J), damping (b) , Breakaway torque (Bt), and coloumb friction torque (Ct)


% Data trimming for lsq:
TestsData1 = load("Data/StepTestsRound1V1.mat");
TestsData2 = load("Data/StepTestsRound2V1.mat");

% Lets use 1:7 for steps, 11:14 for validation
% Which motor? 
% Start with back left
MotorNum = 10; % Back left
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

mdl = 'LSQ\AK109_LSQ';
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
%   data: {"layout":"onright","rightPanelPercent":36.6}
%---
%[output:1ef8beac]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAcUAAAERCAYAAAAKbry9AAAAAXNSR0IArs4c6QAAFXJJREFUeF7t3TFIZMcfB\/BJc5VeEQJyaYIgXG9ITBEkXSCFcBIsrjkbK7GTE4KkOCQgWARECKhgdYUcBFKkFjtz5IoQCAHBTjGkCNxV1\/z\/zP7z\/D\/X3X27jnpv530WQk72jTqf37z97sy8t773\/vvv\/yd4ECBAgAABAuE9oWgUECBAgACB\/wkIRSOBAAECBAj8KyAUDQUCBAgQICAUjQECBAgQIHBZwEzRiCBAgAABAmaKxgABAgQIEDBTNAYIECBAgEBHAcunBgYBAgQIELB8agwQIECAAAHLp8YAAQIECBCwfGoMECBAgACBXgL2FI0PAgQIECBgT9EYIECAAAEC9hSNAQIECBAgkPee4t7eXjg\/Pw8rKytKTYAAAQIEriWQxZ5iDMSHDx+Gw8NDoXitYaARAQIECESBoQ7FqampsLa2Ft6+fduq5u+\/\/y4UjWsCBAgQuLbA0IfiRx99FF6+fBk2NjbC8fGxULz2UNCQAAECBIY6FIvyjY+P9xWKDx48uKj42dmZ6hMgQIAAgUsCjQnFGIirq6thcnKyBbCzsxN2d3cNBwIECBAgcCHQmFCMYbi1tdXag4yzxOI\/Y4EAAQIECBQCjQvF2dnZViB6ECBAgACBdgGhaEwQIECAAIF\/BYSioUCAAAECBHIKxX6qWewpWj7tR8sxBAgQaKZAFjPFfkonFPtRcgwBAgSaLSAUm11\/vSdAgACBkoBQNBwIECBAgIA9RWOAAAECBAhcFjBTNCIIECBAgICZojFAgAABAgTMFH2ijbOAAAECBDoKWD41MAgQIECAgOVTY4AAAQIECFg+tXzqLCBAgAABy6fxT0f5mDdnAgECBAh0E7CnaGwQIECAAAF7isYAAQIECBCwp2hP0VlAgAABAvYU7Sk6CwgQIECgl4A9ReODAAECBAjYUzQGCBAgQICAPUV7is4CAgQIELCnaE\/RWUCAAAEC9hRDCJOTk0EoOhkIECBAQCgKRWcBAQIECPQh4OrTPpAcQoAAAQLNEBCKzaizXhIgQIBAHwJCsQ8khxAgQIBAMwSEYjPqrJcECBAg0IeAUOwDySEECBAg0AwBodiMOuslAQIECPQhIBT7QHIIAQIECDRDQCg2o856SYAAAQJ9CAjFPpAcQoAAAQLNEBCKzaizXhIgQIBAHwJCsQ8khxAgQIBAMwSEYjPqrJcECBAg0IeAUOwDySEECBAg0AyBWobi+vp6mJ6eblXg8PAwrKysdK1G+djT09OwvLwcTk5OrhzvT0c1Y0DrJQECBFIEaheKc3NzYWFhIWxvb7f6Vfx7f3\/\/Sj+XlpbCzMxMWF1dDX\/99VfY2NgIr1+\/DvPz80IxZVRoS4AAgYYK1C4U48xvYmLiYsa3t7cXzs\/PO84W249t\/7pcUzPFho5w3SZAgMAAArULxRiC8VHM9tq\/Lvet00zx+Pi4Y4AWobi4uBjOzs5a\/3kQIECAAIGyQC1DsTwzjLO\/sbGxjkuisSNxuTUG3b1798Lz58\/D5uZmxwoXoVg8ubOzE3Z3d40GAgQIECBwITDUoRgDM4Zd3FM8OjoKvWaVRSiura1dzBTNFp0JBAgQIFD7mWI\/y6fj4+OtC2vKy6Xli3TaL8yxp2jgEyBAgECVQO1miu3Lpd0utBGKVaX1PAECBAgMKlC7UBzkloxOy6ejo6Md71U0Uxx0aDieAAECzROoXSjGEnS7eb+YHR4cHFxcUBNnkg8fPmxVzs37zRvAekyAAIGbFKhlKN5kB4vvZaZ4G6q+JwECBPISEIp51VNvCBAgQCBBQCgm4GlKgAABAnkJCMW86qk3BAgQIJAgIBQT8DQlQIAAgbwEhGJe9dQbAgQIEEgQEIoJeJoSIECAQF4CQjGveuoNAQIECCQICMUEPE0JECBAIC8BoZhXPfWGAAECBBIEhGICnqYECBAgkJeAUMyrnnpDgAABAgkCQjEBT1MCBAgQyEtAKOZVT70hQIAAgQQBoZiApykBAgQI5CUgFPOqp94QIECAQIKAUEzA05QAAQIE8hIQinnVU28IECBAIEFAKCbgaUqAAAECeQkIxbzqqTcECBAgkCAgFBPwNCVAgACBvASEYl711BsCBAgQSBAQigl4mhIgQIBAXgJCMa966g0BAgQIJAgIxQQ8TQkQIEAgLwGhmFc99YYAAQIEEgSEYgKepgQIECCQl4BQzKueekOAAAECCQJCMQFPUwIECBDIS0Ao5lVPvSFAgACBBAGhmICnKQECBAjkJSAU86qn3hAgQIBAgoBQTMDTlAABAgTyEhCKedVTbwgQIEAgQaCWobi+vh6mp6db3To8PAwrKytdu7i0tBQeP37cev7NmzdhdXU1HB0dXTl+cnIybG1thdnZ2XB2dpZApikBAgQI5CpQu1Ccm5sLCwsLYXt7u2Ve\/Ht\/f\/9KDeKxi4uL4cWLF2FzczPEMJ2YmAjLy8vh5OTk0vFCMdchrF8ECBC4OYHahWJ7sO3t7YXz8\/OOs8V47NjYWJifn68UEYqVRA4gQIBA4wVqF4oxBOOjCLr2r4uKjY+Ph42NjXB8fNxzebU4Xig2fqwDIECAQKVALUOxPDPsNhssQvG3334Ln3\/+eRgZGelrTzEut8Y9RfuKlWPDAQQIEGicwNCH4v379y8uromzytHR0Z57ikWFd3Z2wu7ubuMKrsMECBAg0F2glqF43eXT8kU67RfmFMuna2trFzNFs0WnBgECBAiUBWoXiu3Lpb0utGl\/LobikydPwrNnz67clmFP0cAnQIAAgSqB2oXiILdkxHsUZ2ZmLi2flmeZ5c4Lxaqh4HkCBAgQqF0oxpJ0u3m\/uLjm4OCgdV9ifJRv3j89Pe24nxiPE4oGOwECBAhUCdQyFKt+6es8LxSvo6YNAQIEmiUgFJtVb70lQIAAgR4CQtHwIECAAAEC\/woIRUOBAAECBAgIRWOAAAECBAhcFjBTNCIIECBAgICZojFAgAABAgTMFH0YuLOAAAECBDoKWD41MAgQIECAgOVTY4AAAQIECFg+tXzqLCBAgAABy6dbW1thdnZWKDoZCBAgQEAoCkVnAQECBAj0EnChjfFBgAABAgRcaGMMECBAgAABF9rYU3QWECBAgIA9RXuKzgICBAgQsKcYQvBHhp0IBAgQIFAl4EKbKiHPEyBAgEBjBIRiY0qtowQIECBQJSAUq4Q8T4AAAQKNERCKjSm1jhIgQIBAlYBQrBLyPAECBAg0RkAoNqbUOkqAAAECVQJCsUrI8wQIECDQGAGh2JhS6ygBAgQIVAkIxSohzxMgQIBAYwSEYmNKraMECBAgUCUgFKuEPE+AAAECjREQio0ptY4SIECAQJWAUKwS8jwBAgQINEZAKDam1DpKgAABAlUCQrFKyPMECBAg0BgBodiYUusoAQIECFQJCMUqIc8TIECAQGMEahmK6+vrYXp6ulWEw8PDsLKyUlmQubm5sLCwELa3t8P+\/v6V4ycnJ8PW1laYnZ0NZ2dnld\/PAQQIECDQPIHahWI53GI5egVdUa7x8fGwsbERPvjgg1bwCcXmDWQ9JkCAwE0I1C4U4yxxYmIiLC8vh5OTk7C3txfOz897zhaXlpbCzMxMy8NM8SaGhe9BgACBZgrULhRjCMbH\/Px86\/\/tX7eXaWpqKjx9+jQcHBy0grEqFBcXF1vLp5ZQmzng9ZoAAQK9BGoZiuWZYZw5jo2NXYRke2fi8\/Hx66+\/9rWnWLTf2dkJu7u7RgcBAgQIELgQGOpQjPuPjx49Ct9880345JNP+grFtbW1i5mi2aIzgQABAgTKArUMxX6XT+PSapwhbm5uBlefGtgECBAgkCpQu1BsXy7tdqFN3EuMs76RkZErBs+fP28FZfnhlozUoaI9AQIE8heoXShe55aMWCYzxfwHqx4SIEDgtgVqF4qxw91u3i\/uR4xXmrbPBIXibQ8V358AAQL5C9QyFG+D3fLpbaj6ngQIEMhLQCjmVU+9IUCAAIEEAaGYgKcpAQIECOQlIBTzqqfeECBAgECCgFBMwNOUAAECBPISEIp51VNvCBAgQCBBQCgm4GlKgAABAnkJCMW86qk3BAgQIJAgIBQT8DQlQIAAgbwEhGJe9dQbAgQIEEgQEIoJeJoSIECAQF4CQjGveuoNAQIECCQICMUEPE0JECBAIC8BoZhXPfWGAAECBBIEhGICnqYECBAgkJeAUMyrnnpDgAABAgkCQjEBT1MCBAgQyEtAKOZVT70hQIAAgQQBoZiApykBAgQI5CUgFPOqp94QIECAQIKAUEzA05QAAQIE8hIQinnVU28IECBAIEFAKCbgaUqAAAECeQkIxbzqqTcECBAgkCAgFBPwNCVAgACBvASEYl711BsCBAgQSBAQigl4mhIgQIBAXgJCMa966g0BAgQIJAgIxQQ8TQkQIEAgLwGhmFc99YYAAQIEEgSEYgKepgQIECCQl4BQzKueekOAAAECCQJCMQFPUwIECBDIS0Ao5lVPvSFAgACBBIFahuL6+nqYnp5udevw8DCsrKx07OLU1FRYW1sLIyMjredPT0\/D8vJyODk5uXL85ORk2NraCrOzs+Hs7CyBTFMCBAgQyFWgdqE4NzcXFhYWwvb2dsu8+Pf+\/v6lGoyPj4eNjY1wfHzcCs3i69evX4f5+XmhmOuI1S8CBAjcokDtQjHOEicmJi5mfHt7e+H8\/LzrbLFs0962\/JyZ4i2OIt+aAAECmQjULhRjCMZHMdtr\/7qXu1DMZFTqBgECBN6RQC1DsTwzjEE3NjbWcUm0bFbsL7569arjrLKYKS4uLrb2FO0rvqMR58cSIECgxgJZhGKxnxidqy60KWqxs7MTdnd3a1wavxoBAgQI3LVALUNxkOXTfgIxfr9iphivVi1mimaLdz3c\/DwCBAjUW6B2odi+XNrrQpuqK07L9C60qfdA9NsRIECgDgK1C8V+b8mIeDEwR0dHuy6ZCsU6DDG\/AwECBIZHoHahGOm63bxfzAwPDg7CL7\/8cunG\/YL8zZs3YXV1NRwdHV2qgpni8AxKvykBAgTelUAtQ\/E2MITibaj6ngQIEMhLQCjmVU+9IUCAAIEEAaGYgKcpAQIECOQlIBTzqqfeECBAgECCgFBMwNOUAAECBPISEIp51VNvCBAgQCBBQCgm4GlKgAABAnkJCMW86qk3BAgQIJAgIBQT8DQlQIAAgbwEhGJe9dQbAgQIEEgQEIoJeJoSIECAQF4CQjGveuoNAQIECCQICMUEPE0JECBAIC8BoZhXPfWGAAECBBIEhGICnqYECBAgkJeAUMyrnnpDgAABAgkCQjEBT1MCBAgQyEtAKOZVT70hQIAAgQQBoZiApykBAgQI5CUgFPOqp94QIECAQIKAUEzA05QAAQIE8hIQinnVU28IECBAIEFAKCbgaUqAAAECeQkIxbzqqTcECBAgkCAgFBPwNCVAgACBvASEYl711BsCBAgQSBAQigl4mhIgQIBAXgJCMa966g0BAgQIJAgIxQQ8TQkQIEAgLwGhmFc99YYAAQIEEgSEYgKepgQIECCQl4BQzKueekOAAAECCQJCMQFPUwIECBDIS0Ao5lVPvSFAgACBBAGhmICnKQECBAjkJTD0obi+vh6mp6dbVTk8PAwrKysdKzQ5ORm2trbC7OxsODs7y6uKekOAAAECNyIw1KE4NzcXFhYWwvb2dguj+Pf+\/v4VHKHYebw8ePAgfPXVV+Hnn3\/2ZqFExMV4GeQV1njJZ7wMdSjGWeLExERYXl4OJycnYW9vL5yfn3ecLQrFzoOWC5dBXvyNF+Ml9\/Ey1KEYQzA+5ufnW\/9v\/7pcvOJkXlxcNCNqmxHFZWUul0\/1+M6fy9WXPy7dZ0TGS\/fxMkzbVkMfiuWZYZw5jo2NXYRkuUTxZF5dXQ0xHD0IECBA4G4EXr161XrTPSyPxoRiLEgMxvifBwECBAjcjUC8sHGYLm4c+lDsd\/n0bsrvpxAgQIDAMAsMdSi2L5f2utBmmIvkdydAgACBuxEY6lAc5JaMu+H0UwgQIEBgmAWGOhQjfL837w9zkfzuBAgQIHA3AkMfinfD5KcQIECAQBMEhGITqqyPBAgQINCXQCNCMV6A8\/DhwxbI8+fPw+bmZl84uRwU917jfUL37t0Lp6enF58A1Kl\/Zau3b9+2bmDv9LF5OdgM4lL0d3x8PGxsbITj4+Oun7PbNJupqamwtrYWRkZGsj\/HBhkz5WNzP5d6jfninDk4OBiK197sQ3FpaSnMzMy0btz\/9NNPL\/59dHSUw2tXZR\/KL+I\/\/PBDzxf09o\/Ni1\/HDzuIdrl5DeJSRi72sHt9+HxlUWp+wCA2xbGvX79ufWhG+eK33N5MDeJSvFH46aefWkGQ87nUTyB++OGHQzMhyT4Uyx\/9NmzvWG7itbP9RSq+Sfjiiy96zhaLn5vzC9x1XOIL3bffftuaccdP6ej2F1luom7v8nsMYhOPffLkSXj27Fl2b5zaazCoS\/kPFOR8LnUbq8VM+e+\/\/w73798PxRuEdzm2+\/nZWYdi+1JXU5a+yoUvz5TjbK\/9616DJOcT+Tou8Q3WH3\/80VpxyHn5dBCb9tWFfl50hvWYQVw6zRTLf7xgWA0G+b2\/\/PLL1uH\/\/PNPa3ldKA6id0vHdpoZNu0G\/\/aZ4SDv7KPV6OhoX7PKWyrhrX3bQV2i26NHj8L3338fnj59mn0ollcTeo2Z4gM0YqFy37cfdMwUrz9x6fDPP\/\/s+JnMtzbAa\/SN298g1OhX6\/irNGKmWN7gFYr9LXfFF7vPPvss2wttBnmBiy9u3333Xfjxxx\/Dy5cvs7\/QZhCbYo+1uIAttv3666+zHDeDuBRLhy9evGjtKQ6yQlP30Bj09xOKg4rd4vGWT8OVk7GfkzP3QIxDbpClsHjsxx9\/3Hqn34Ql+EFs2pdPc\/bhcr0Xa6F4Pbdba1WeGTb1QpvyhRBVF9o05Sq59iXBXi7l21TKAzXXJbFBbNrdcj7HBnFp0puFqhdvoVgldMfPuyXj\/\/fVVd2SkfPSV\/uwG+Ty+nLbnGdCRT8HsWl\/wetnJeKOXwJu7McN4tJp+TTXZeUqYKFYJfQOnnfzfveb98t\/aaTbjCjXDzzodSN2tz9Y3YRQjKfoIDblm\/dzv0l9EJf4BuHx48etV7zcXXq9rAvFdxB6fiQBAgQIELgJgayvPr0JIN+DAAECBJojIBSbU2s9JUCAAIEKAaFoiBAgQIAAgX8FhKKhQIAAAQIEhKIxQIAAAQIELguYKRoRBAgQIEDATNEYIECAAAECZorGAAECBAgQ6Chg+dTAIECAAAEClk+NAQIECBAgYPnUGCBAgAABApZPjQECBAgQINBL4L+2BS9p4pOH8AAAAABJRU5ErkJggg==","height":273,"width":453}}
%---
