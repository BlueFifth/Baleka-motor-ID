clear;
clc;
% Initial guesses 
hold off %[output:1ef8beac]



% J = 1.59;
% J = 1.4465;
% b = 1.49;
% Bt = 0.4787; % Breakaway friction torque
% Ct = 0.3169; % Coulomb torque

J = 1.4465;
b = 1.1;
Bt = 0.5143; % Breakaway friction torque
Ct = 0.2853; % Coulomb torque

% J = 1.4465;
% b = 1.3;
% Bt = 0.5521; % Breakaway friction torque
% Ct = 0.2682; % Coulomb torque
q=0;
StartVel = 1;
StepVel = 1;
Kp = 0;
Kd = 1;
StepTime = 5;


%%

%%
[b, Bt, Ct] = AK10lsqPos %[output:7fe7d649]
%%
function [b, Bt, Ct] = AK10lsqPos

SimTime = 3;
% Get comparison data
data = cell(1,8);
load("StepTests1.mat");
steps = cell(1, 13);
for i=1:7
    steps{i} = timeseries2timetable(data{i}{1}.Values);
    steps{i} = steps{i}.M1_pos(5001:5001 + SimTime*1000);
end
load("ProportionalSteps.mat");
for i=8:13
    steps{i} = timeseries2timetable(data{i-7}{2}.Values);
    steps{i} = steps{i}.M1_pos(5001:5001 +SimTime*1000);
end
stepq = [steps{1}; steps{3};steps{4};steps{5}; steps{6}; steps{7};
    steps{8}; steps{9}; steps{10}; steps{11}; steps{12}; steps{13}];

mdl = 'AK109_LSQ';
open_system(mdl) %Load the model
in = Simulink.SimulationInput(mdl); % Create simulation input object
in = in.setModelParameter("StopTime",num2str(SimTime));
in = in.setVariable('StepTime', 0, 'Workspace',mdl);


% J = 1.59;
% b = 1.49;
% Bt = 0.7; % Breakaway friction torque
% Ct = 0.144; % Coulomb torque
% J = 1.4465;
% b = 1.3;
% Bt = 0.5521; % Breakaway friction torque
% Ct = 0.2682; % Coulomb torque
J = 1.4465;
b = 1.1;
Bt = 0.5143; % Breakaway friction torque
Ct = 0.2853; % Coulomb torque


pid0 = [b, Bt, Ct];

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
b = pid(1); Bt = pid(2); Ct = pid(3);
% Bt = pid(1); Ct = pid(2);
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
      % in = in.setVariable('Ip',pid(1),'Workspace',mdl);
      % in = in.setVariable('Ii',pid(2),'Workspace',mdl);
      % in = in.setVariable('J',pid(1),'Workspace',mdl);
      in = in.setVariable('b', pid(1), 'Workspace', mdl);
      in = in.setVariable('Bt',pid(2),'Workspace',mdl);
      in = in.setVariable('Ct', pid(3), 'Workspace', mdl);

      
      % Step 1
      in = in.setVariable('q', 3, 'Workspace', mdl);
      in = in.setVariable('Kp', 1, 'Workspace', mdl);
      in = in.setVariable('Kd', 0, 'Workspace', mdl);
      out = sim(in);
      simdata = out.yout{1}.Values.Data;
      % plot(simdata)
      
      % % % Step 2
      % % in = in.setVariable('q', 5, 'Workspace', mdl);
      % % in = in.setVariable('Kp', 1, 'Workspace', mdl);
      % % in = in.setVariable('Kd', 0, 'Workspace', mdl);
      % % out = sim(in);
      % % simdata = [simdata; out.yout{1}.Values.Data];
      % 
      % Step 3
      in = in.setVariable('q', 1, 'Workspace', mdl);
      in = in.setVariable('Kp', 1, 'Workspace', mdl);
      in = in.setVariable('Kd', 0, 'Workspace', mdl);
      out = sim(in);
      simdata = [simdata; out.yout{1}.Values.Data];

      % Step 4
      in = in.setVariable('q', 1, 'Workspace', mdl);
      in = in.setVariable('Kp', 5, 'Workspace', mdl);
      in = in.setVariable('Kd', 0, 'Workspace', mdl);
      out = sim(in);
      simdata = [simdata; out.yout{1}.Values.Data];

      % Step 5
      in = in.setVariable('q', -4, 'Workspace', mdl);
      in = in.setVariable('Kp', 5, 'Workspace', mdl);
      in = in.setVariable('Kd', 1, 'Workspace', mdl);
      out = sim(in);
      simdata = [simdata; out.yout{1}.Values.Data];

      % Step 6
      in = in.setVariable('q', -1, 'Workspace', mdl);
      in = in.setVariable('Kp', 5, 'Workspace', mdl);
      in = in.setVariable('Kd', 1, 'Workspace', mdl);
      out = sim(in);
      simdata = [simdata; out.yout{1}.Values.Data];

      % Step 7
      in = in.setVariable('q', 6, 'Workspace', mdl);
      in = in.setVariable('Kp', 5, 'Workspace', mdl);
      in = in.setVariable('Kd', 1, 'Workspace', mdl);
      out = sim(in);
      simdata = [simdata; out.yout{1}.Values.Data];

      % Step 8
      in = in.setVariable('q', 3, 'Workspace', mdl);
      in = in.setVariable('Kp', 1, 'Workspace', mdl);
      in = in.setVariable('Kd', 0, 'Workspace', mdl);
      out = sim(in);
      simdata = [simdata; out.yout{1}.Values.Data];

      % Step 9
      in = in.setVariable('q', 2, 'Workspace', mdl);
      in = in.setVariable('Kp', 1, 'Workspace', mdl);
      in = in.setVariable('Kd', 0, 'Workspace', mdl);
      out = sim(in);
      simdata = [simdata; out.yout{1}.Values.Data];

      % Step 10
      in = in.setVariable('q', 1, 'Workspace', mdl);
      in = in.setVariable('Kp', 5, 'Workspace', mdl);
      in = in.setVariable('Kd', 0, 'Workspace', mdl);
      out = sim(in);
      simdata = [simdata; out.yout{1}.Values.Data];
      
      % Step 11
      in = in.setVariable('q', 4, 'Workspace', mdl);
      in = in.setVariable('Kp', 1, 'Workspace', mdl);
      in = in.setVariable('Kd', 0, 'Workspace', mdl);
      out = sim(in);
      simdata = [simdata; out.yout{1}.Values.Data];
      
      % Step 12
      in = in.setVariable('q', 2, 'Workspace', mdl);
      in = in.setVariable('Kp', 2, 'Workspace', mdl);
      in = in.setVariable('Kd', 0, 'Workspace', mdl);
      out = sim(in);
      simdata = [simdata; out.yout{1}.Values.Data];
      
      % Step 13
      in = in.setVariable('q', 2, 'Workspace', mdl);
      in = in.setVariable('Kp', 3, 'Workspace', mdl);
      in = in.setVariable('Kd', 0, 'Workspace', mdl);
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
%   data: {"layout":"onright","rightPanelPercent":44.1}
%---
%[output:1ef8beac]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAhYAAAGQCAYAAADyRimLAAAAAXNSR0IArs4c6QAAH4NJREFUeF7t3T9PXOnZwOEnJS1KE0tORb3RNqB8BbuGDorECIkmzSIhmRIkJLZJg4RAKaCDNt6vsIJmpa2pstJuE9G6zKtDcnjH4\/nH8MzNzH0ul\/bMeXiucw\/++cwZ\/Lv\/\/Oc\/\/yl+ESBAgAABAgQqCPxOWFRQdAgCBAgQIEDgUUBYGAQCBAgQIECgmoCwqEbpQAQIECBAgICwMAMECBAgQIBANQFhUY3SgQgQIECAAAFhYQYIECBAgACBagLCohqlAxEgQIAAAQLCwgwQIECAAAEC1QSERTVKByJAgAABAgSEhRkgQIAAAQIEqgkIi2qUDkSAAAECBAgICzNAgAABAgQIVBMQFtUoHYgAAQIECBAQFmaAAAECBAgQqCYgLKpROhABAgQIECAgLMwAAQIECBAgUE1AWFSjdCACBAgQIEBAWJgBAgQIECBAoJqAsKhG6UAECBAgQICAsDADBAgQIECAQDUBYVGN0oEIECBAgAABYWEGCBAgQIAAgWoCwqIapQMRIECAAAECnQiL+\/v7sr+\/X46Pj8vKyoqzToAAAQIECMxIIH1YPDw8lJ2dnfLvf\/+7nJ+fC4sZDZLDEiBAgACBRiB1WNze3patra3HM\/3mzRthYeYJECBAgMCMBdKGRRsVh4eH5Y9\/\/OPjWyGuWMx4mhyeAAECBDovkDYses9sExnCovOzDoAAAQIEAgSERQCyJQgQIECAQFcEhMWAM725ufnF715dXXVlHuyTAAECBAi8SEBY9PE1UXF3d1dWV1ef\/kRYvGjGPJkAAQIEOiQgLAaERfNbYqJDrwJbJUCAAIFqAsJCWFQbJgciQIAAAQLCQlh4FRAgQIAAgWoCwkJYVBsmByJAgAABAp0Ii+ec5vYTIe6xeI6axxIgQIAAgf8KCAtXLLwWCBAgQIBANQFhISyqDZMDESBAgAABYSEsvAoIECBAgEA1AWEhLKoNkwMRIECAAAFhISy8CggQIECAQDUBYSEsqg2TAxEgQIAAAWEhLLwKCBAgQIBANQFhISyqDZMDESBAgAABYSEsvAoIECBAgEA1AWEhLKoNkwMRIECAAAFhISy8CggQIECAQDUBYSEsqg2TAxEgQIAAAWEhLLwKCBAgQIBANQFhISyqDZMDESBAgAABYSEsvAoIECBAgEA1AWEhLKoNkwMRIECAAAFhISy8CggQIECAQDUBYSEsqg2TAxEgQIAAAWEhLLwKCBAgQIBANQFhISyqDZMDESBAgAABYSEsvAoIECBAgEA1AWEhLKoNkwMRIECAAAFhISy8CggQIECAQDUBYSEsqg2TAxEgQIAAAWEhLLwKCBAgQIBANQFhISyqDZMDESBAgAABYSEsvAoIECBAgEA1AWEhLKoNkwMRIECAAAFhISy8CggQIECAQDUBYSEsqg2TAxEgQIAAAWEhLLwKCBAgQIBANQFhISyqDZMDESBAgAABYSEsvAoIECBAgEA1AWEhLKoNkwMRIECAAAFhISy8CggQIECAQDUBYSEsqg2TAxEgQIAAAWEhLLwKCBAgQIBANQFhISyqDZMDESBAgAABYSEsvAoIECBAgEA1AWEhLKoNkwMRIECAAAFhISy8CggQIECAQDUBYSEsqg2TAxEgQIAAAWEhLLwKCBAgQIBANQFhISyqDZMDESBAgAABYSEsvAoIECBAgEA1AWEhLKoNkwMRIECAAAFhISy8CggQIECAQDUBYSEsqg2TAxEgQIAAAWEhLLwKCBAgQIBANQFhISyqDZMDESBAgAABYSEsvAoIECBAgEA1AWEhLKoNkwMRIECAAIGFCIvb29uytbX1dLYODw\/L+vr62LN3c3NTDg4OnvW8zc3Nx8dfXV2NPb4HECBAgAABAl8KzH1YNFGxv79fzs\/Py8rKSrm\/vy\/b29tld3d3ZFw0UXF6evrs5wkLLxECBAgQIDC9wFyHxcPDQ9nZ2Smrq6tlb2\/vaZdNNFxfX5ezs7OyvLz81e6HPe\/k5KTc3d0NfV5zIGEx\/TB5JgECBAgQmOuwaK9OHB8fl7W1taezNez32wcIC4NNgAABAgReR2Cuw6L\/bZCWaJK3Q4a9FfLu3bsvrn70s7ti8TqDaFUCBAgQyCGwkGHRXpHY2NgYeZ9FGyC\/\/vrr49m6vLz84srHoFMoLHIMtl0QIECAwOsIpAyLz58\/l48fP5Zffvnl6X6KNkbevn1bjo6OytLS0kDxJiya+zCa+zraXz4h8jrDaVUCBAgQWDyBhQyLcW+FtB9P7b9CMe7ejOb0CYvFG2JfMQECBAjMj8Bch8W0N28OuzdjkrdQvBUyP8PpKyFAgACBxROY67AYFgLjPm760isWzWn09sfiDbOvmAABAgReX2Cuw6LhaX96Zvu2xri3QZrnvPQeC2Hx+oPpKyBAgACBxRSY+7BoWMf9SO\/+j5a2p6L5gVgXFxdPZ+bDhw8jP2raPNBbIYs5yL5qAgQIEJgPgYUIi0gqYRGpbS0CBAgQyCYgLPrOqLDINuL2Q4AAAQKRAsJCWETOm7UIECBAILmAsBAWyUfc9ggQIEAgUkBYCIvIebMWAQIECCQXEBbCIvmI2x4BAgQIRAoIC2EROW\/WIkCAAIHkAsJCWCQfcdsjQIAAgUgBYSEsIufNWgQIECCQXEBYCIvkI257BAgQIBApICyEReS8WYsAAQIEkgsIC2GRfMRtjwABAgQiBYSFsIicN2sRIECAQHIBYSEsko+47REgQIBApICwEBaR82YtAgQIEEguICyERfIRtz0CBAgQiBQQFsIict6sRYAAAQLJBYSFsEg+4rZHgAABApECwkJYRM6btQgQIEAguYCwEBbJR9z2CBAgQCBSQFgIi8h5sxYBAgQIJBcQFsIi+YjbHgECBAhECggLYRE5b9YiQIAAgeQCwkJYJB9x2yNAgACBSAFhISwi581aBAgQIJBcQFgIi+QjbnsECBAgECkgLIRF5LxZiwABAgSSCwgLYZF8xG2PAAECBCIFhIWwiJw3axEgQIBAcgFhISySj7jtESBAgECkgLAQFpHzZi0CBAgQSC4gLIRF8hG3PQIECBCIFBAWwiJy3qxFgAABAskFhIWwSD7itkeAAAECkQLCQlhEzpu1CBAgQCC5gLAQFslH3PYIECBAIFJAWAiLyHmzFgECBAgkFxAWwiL5iNseAQIECEQKCAthETlv1iJAgACB5ALCQlgkH3HbI0CAAIFIAWEhLCLnzVoECBAgkFxAWAiL5CNuewQIECAQKSAshEXkvFmLAAECBJILCAthkXzEbY8AAQIEIgWEhbCInDdrESBAgEByAWEhLJKPuO0RIECAQKSAsBAWkfNmLQIECBBILiAshEXyEbc9AgQIEIgUEBbCInLerEWAAAECyQWEhbBIPuK2R4AAAQKRAsJCWETOm7UIECBAILmAsBAWyUfc9ggQIEAgUkBYCIvIebMWAQIECCQXWIiwuL29LVtbW0+n4vDwsKyvr489Nf3Pe\/\/+fTk6OipLS0tDn7u5ufn4Z1dXV2OP7wEECBAgQIDAlwJzHxZNHOzv75fz8\/OysrJS7u\/vy\/b2dtnd3R0ZFzc3N+Xg4KBcXl6WtbW18vDwUHZ2dsrbt29HxoWw8BIhQIAAAQLTC8x1WLQxsLq6Wvb29p522UTD9fV1OTs7K8vLy1\/tvn3exsbGF\/HRHymD2ITF9MPkmQQIECBAYK7Dor06cXx8\/HjVof017PfbP28C4vvvvx8aHqNOu7DwoiBAgAABAtMLzHVYDLvCMO7tkOaKxo8\/\/li+++678re\/\/a38\/PPPj0IfPnz44sqHKxbTD45nEiBAgACBQQILGRbD3upoN3hyclIuLi7Kmzdvnu7NcI+FFwABAgQIEJi9QOqwaG\/c7H2LpPl0Sf\/v9zI3b4Xc3d2V5r6O9pdPiMx+EK1AgAABAjkEFjIsxr0V0lyx+OGHH56uVrSnatyVjuZxwiLHYNsFAQIECLyOwFyHxbQ3bzb3WJyenk4dFs2pcJXidQbSqgQIECCw2AJzHRbDrjCM+7jpsCsaPm662MPqqydAgACB+ReY67Bo+Pp\/0NW4t0Fa8v63Q4b9TIz+U+TjpvM\/tL5CAgQIEJhfgbkPi4Zu3I\/0HvbWRxslLb+Pm87vIPrKCBAgQCCHwEKERSS1KxaR2tYiQIAAgWwCwqLvjAqLbCNuPwQIECAQKSAshEXkvFmLAAECBJILCAthkXzEbY8AAQIEIgWEhbCInDdrESBAgEByAWEhLJKPuO0RIECAQKSAsBAWkfNmLQIECBBILiAshEXyEbc9AgQIEIgUEBbCInLerEWAAAECyQWEhbBIPuK2R4AAAQKRAsJCWETOm7UIECBAILmAsBAWyUfc9ggQIEAgUkBYCIvIebMWAQIECCQXEBbCIvmI2x4BAgQIRAoIC2EROW\/WIkCAAIHkAsJCWCQfcdsjQIAAgUgBYSEsIufNWgQIECCQXEBYCIvkI257BAgQIBApICyEReS8WYsAAQIEkgsIC2GRfMRtjwABAgQiBYSFsIicN2sRIECAQHIBYSEsko+47REgQIBApICwEBaR82YtAgQIEEguICyERfIRtz0CBAgQiBQQFsIict6sRYAAAQLJBYSFsEg+4rZHgAABApECwkJYRM6btQgQIEAguYCwEBbJR9z2CBAgQCBSQFgIi8h5sxYBAgQIJBcQFsIi+YjbHgECBAhECggLYRE5b9YiQIAAgeQCwkJYJB9x2yNAgACBSAFhISwi581aBAgQIJBcQFgIi+QjbnsECBAgECkgLIRF5LxZiwABAgSSCwgLYZF8xG2PAAECBCIFhIWwiJw3axEgQIBAcgFhISySj7jtESBAgECkgLAQFpHzZi0CBAgQSC4gLIRF8hG3PQIECBCIFBAWwiJy3qxFgAABAskFhIWwSD7itkeAAAECkQLCQlhEzpu1CBAgQCC5gLAQFslH3PYIECBAIFJAWAiLyHmzFgECBAgkFxAWwiL5iNseAQIECEQKCAthETlv1iJAgACB5ALCQlgkH3HbI0CAAIFIAWEhLCLnzVoECBAgkFxAWAiL5CNuewQIECAQKSAshEXkvFmLAAECBJILCAthkXzEbY8AAQIEIgWEhbCInDdrESBAgEByAWEhLJKPuO0RIECAQKTAQoTF7e1t2draenI5PDws6+vrz3K6ubkp19fX5ezsrCwvLw997ubm5uOfXV1dPev4HkyAAAECBAiUMvdh0UTF\/v5+OT8\/LysrK+X+\/r5sb2+X3d3dieOifc7vf\/97YWHqCRAgQIDADAXmOiweHh7Kzs5OWV1dLXt7e08Mk159aJ7w+fPn8vHjx\/Lp06fyzTffCIsZDpNDEyBAgACBuQ6L9krD8fFxWVtbezpbw35\/0OlsIuTHH38sf\/rTn8o\/\/\/lPYWHmCRAgQIDADAXmOiz63wZpHSZ9O6T3+T\/99JN7LGY4SA5NgAABAgQagYUMi\/Ytko2NjaH3WfQ\/ZtK3T5qbN+\/u7h7ffml\/uZHTi4UAAQIECEwmkDYsTk5Oym+\/\/VaOjo7K0tJSERaTDYRHESBAgACBlwgsZFiMeytk0FsozwmLBtRVipeMlecSIECAQFcF5jospr15s7lacXFxMfScjvo5GH6ORVdfCvZNgAABAjUE5josht1LMenVh16gSZ8jLGqMlWMQIECAQFcF5josmpPSBMHBwUG5vLx8\/MjpuLdBhp1IYdHVEbdvAgQIEIgUmPuwaDDG\/UjvJhpOT0+ffjrnIEBhETlW1iJAgACBrgosRFhEnhxvhURqW4sAAQIEsgkIi74zKiyyjbj9ECBAgECkgLAQFpHzZi0CBAgQSC4gLIRF8hG3PQIECBCIFBAWwiJy3qxFgAABAskFhIWwSD7itkeAAAECkQLCQlhEzpu1CBAgQCC5gLAQFslH3PYIECBAIFJAWAiLyHmzFgECBAgkFxAWwiL5iNseAQIECEQKCAthETlv1iJAgACB5ALCQlgkH3HbI0CAAIFIAWEhLCLnzVoECBAgkFxAWAiL5CNuewQIECAQKSAshEXkvFmLAAECBJILCAthkXzEbY8AAQIEIgWEhbCInDdrESBAgEByAWEhLJKPuO0RIECAQKSAsBAWkfNmLQIECBBILiAshEXyEbc9AgQIEIgUEBbCInLerEWAAAECyQWEhbBIPuK2R4AAAQKRAsJCWETOm7UIECBAILmAsBAWyUfc9ggQIEAgUkBYCIvIebMWAQIECCQXEBbCIvmI2x4BAgQIRAoIC2EROW\/WIkCAAIHkAsJCWCQfcdsjQIAAgUgBYSEsIufNWgQIECCQXEBYCIvkI257BAgQIBApICyEReS8WYsAAQIEkgsIC2GRfMRtjwABAgQiBYSFsIicN2sRIECAQHIBYSEsko+47REgQIBApICwEBaR82YtAgQIEEguICyERfIRtz0CBAgQiBQQFsIict6sRYAAAQLJBYSFsEg+4rZHgAABApECwkJYRM6btQgQIEAguYCwEBbJR9z2CBAgQCBSQFgIi8h5sxYBAgQIJBcQFsIi+YjbHgECBAhECggLYRE5b9YiQIAAgeQCwkJYJB9x2yNAgACBSAFhISwi581aBAgQIJBcQFgIi+QjbnsECBAgECkgLIRF5LxZiwABAgSSCwgLYZF8xG2PAAECBCIFhIWwiJw3axEgQIBAcgFhISySj7jtESBAgECkgLAQFpHzZi0CBAgQSC4gLIRF8hG3PQIECBCIFBAWwiJy3qxFgAABAskFFiIsbm9vy9bW1tOpODw8LOvr6yNPzefPn8vHjx\/Lp0+fnvW8zc3Nx8dfXV0lP\/W2R4AAAQIE6gvMfVg0UbG\/v1\/Oz8\/LyspKub+\/L9vb22V3d3doXDw8PJSdnZ3y9u3bcnR0VJaWlp6e9+7du7K3tzdUUljUHzJHJECAAIHuCMx1WLSBsLq6+kUM3NzclOvr63J2dlaWl5e\/Olv9MdI+YNzzmscJi+4Mv50SIECAQH2BuQ6L9urE8fFxWVtbe9r9sN8fx9OExenp6dPVj0GPFxbjFP05AQIECBAYLjDXYTHsysMkb4cM2vLJyUm5u7sbeqXDFQsvFQIECBAg8DKBhQyL9i2SjY2NsTdxtjztDaDjbvxsrlg08dG8\/dL+ciPny4bMswkQIECgOwKdCIv2Cse33377dDPnsFMsLLoz\/HZKgAABAvUFFjIsnvNWyHOiwlsh9QfMEQkQIECgWwJzHRYvvXmzffvj\/fv3Y69UtKfdzZvdegHYLQECBAjUFZjrsBh2L8UkHxtto+LDhw8jf25FP6ewqDtgjkaAAAEC3RKY67BoTkUTEQcHB+Xy8vLxI6eTvA3SPmbcD8MadKqFRbdeAHZLgAABAnUF5j4smu2O+5He\/T+fovlY6cXFxVCpNlKERd1hcjQCBAgQILAQYRF5mlyxiNS2FgECBAhkExAWfWdUWGQbcfshQIAAgUgBYSEsIufNWgQIECCQXEBYCIvkI257BAgQIBApICyEReS8WYsAAQIEkgsIC2GRfMRtjwABAgQiBYSFsIicN2sRIECAQHIBYSEsko+47REgQIBApICwEBaR82YtAgQIEEguICyERfIRtz0CBAgQiBQQFsIict6sRYAAAQLJBYSFsEg+4rZHgAABApECwkJYRM6btQgQIEAguYCwEBbJR9z2CBAgQCBSQFgIi8h5sxYBAgQIJBcQFsIi+YjbHgECBAhECggLYRE5b9YiQIAAgeQCwkJYJB9x2yNAgACBSAFhISwi581aBAgQIJBcQFgIi+QjbnsECBAgECkgLIRF5LxZiwABAgSSCwgLYZF8xG2PAAECBCIFhIWwiJw3axEgQIBAcgFhISySj7jtESBAgECkgLAQFpHzZi0CBAgQSC4gLIRF8hG3PQIECBCIFBAWwiJy3qxFgAABAskFhIWwSD7itkeAAAECkQLCQlhEzpu1CBAgQCC5gLAQFslH3PYIECBAIFJAWAiLyHmzFgECBAgkFxAWwiL5iNseAQIECEQKCAthETlv1iJAgACB5ALCQlgkH3HbI0CAAIFIAWEhLCLnzVoECBAgkFxAWAiL5CNuewQIECAQKSAshEXkvFmLAAECBJILCAthkXzEbY8AAQIEIgWEhbCInDdrESBAgEByAWEhLJKPuO0RIECAQKSAsBAWkfNmLQIECBBILiAshEXyEbc9AgQIEIgUEBbCInLerEWAAAECyQWEhbBIPuK2R4AAAQKRAsJCWETOm7UIECBAILmAsBAWyUfc9ggQIEAgUkBYCIvIebMWAQIECCQXEBbCIvmI2x4BAgQIRAoIC2EROW\/WIkCAAIHkAsJCWCQfcdsjQIAAgUgBYSEsIufNWgQIECCQXEBYCIvkI257BAgQIBApICyEReS8WYsAAQIEkgukDovb29uytbX1dAoPDw\/L+vr6yFO6ubn5+OdXV1fJT73tESBAgACB+gJpw6KJiv39\/XJ+fl5WVlbK\/f192d7eLru7uyPjQljUHzJHJECAAIHuCKQMi4eHh7Kzs1NWV1fL3t7e09m8ubkp19fX5ezsrCwvLw88y8KizvBzrOPYHIUly3oC9Y5kLutYZnRMGRbt1Ynj4+Oytrb2dPaH\/X7veGQ8yXXG\/3lH4fg8r1GPZsmynkC9I5nLOpYZHVOGRf\/bIO3pn+TtkIwnuc74P+8oHJ\/nJSzqebFkGSNQZ5WM3ys7FRbtWyQbGxtD77NoTvLd3d3j2yh+TS\/QGDa\/OE5v2D6T5csNWdYzZFnXsv37JtMHBoTFgBlpC7Lu+DgaAQIECBD4WiBTVDS761RYTPJWiKEnQIAAAQIEphdIGRYvuXlzekrPJECAAAECBFKGxbB7KSb5uKmRIECAAAECBKYXSBkWDUcTEQcHB+Xy8vLxI6feBpl+SDyTAAECBAhMKpA2LBqAaX6k96RwHkeAAAECBAh8LZA6LJxwAgQIECBAIFZAWMR6W40AAQIECKQWEBapT6\/NESBAgACBWAFhEettNQIECBAgkFpAWPzv9H7+\/Ll8\/PixfPr06fF3vvnmm5H\/C2rqqZhwcycnJ+Xi4uLx0W\/evHn6L+pHPb3\/hlrO\/9WaxrLXeZIfVz\/haV3oh03j2P\/abwDaT5MtNMYLv\/hpLNs5\/Pnnn30ffaZ\/49386v0fuZ95iLl5uLAopbTfWP7whz88ndTmJDc\/w33Uf7E+N2fxFb6Qxue3334rR0dHZWlp6fHjvaenpyPjov8jwO1fqD\/88MNEUfIK2wxZchrL\/i+s\/Uvg8PBw6P+DE7KZV1xkGsf2L8K3b99+Mcu9H1V\/xS292tIvsWz+f6D2L0ffRyc7he33xg8fPgiLycjm\/1GD\/lL0L8Dh56296tD7r7pBcdZ7hGF\/3nXnaSz7z0zvVaCuhsW0joN+aN64WZ7\/72gv+wpfYtn\/jws\/P2j0uei\/WiYsXja7c\/Xs\/jpvv7hhvz9XX\/wrfDHDfoLpND\/ZtA2L3n\/lvMKWXm3Jl1q2fn\/5y1\/KP\/7xjzLqf+59tU0GLDyNY\/tN\/c9\/\/nNnr\/IMOjXTWDbHGfQPNGExfPjb+fvll1\/K3\/\/+9\/L999+X3qvmAS+bmS3R+bdCRv3rxGW8wXM3LLgmeTuk\/4hd\/8bzEsve2f3rX\/9adnZ2OhsW0zi2Ufbdd9+Vf\/3rX48\/qbf5Nen9QjP7rvzKB57GsvmSB\/0joTlW19\/qnOR0ZrtKJiz+d9PmoFKc5l\/gkwzRoj9m2Dee5hLq\/v7+xPdL9BZ7V+9leYll73w2MyUs\/v+en\/Y1Nmom26j99ddfS+8l6EH3Ai36a\/Y5X\/9LZrJZp\/emz\/fv3z\/du\/Kcr6FrjxUWyc74qBMqLJ53xeK5YdF+A+ryHfjTfhPv\/x98u36vyjSOreG33377xV9+7feEZvrbm5OTfdsbuZ1pLJsDtvdm9N7n0\/VIm3RuhMWkUgvyOG+FPP9ETXuptHclUfFfjWksB82ssPjyU0rtrI16e64Ni3fv3n11J36X\/1HxkpkcFGPuVRv\/PVZYjDdauEe4efN5p2zam7uaVXrvgu7ylYrev\/iur6+\/+ljzqL\/Yei\/hDzpzXfzZINPM5Kgbh7scFtNYuvL7vO+h\/Y8WFi\/zm8tnD3ohdf1fgKNO1KC3PCZ5YbSP+emnnya+D2MuB6biFzWtZf+X0PV5ndZx0A3ak8xyxRGYu0NNYznq7SNXLMaf4mwz1\/mbN5tTPuiH5PhEyPAXw6CbLif5RIg7xL82ndZSWHwpMK3joLdDJpnl8X9VLO4jprV0j8X051xYTG8318\/s\/0ElXbyc\/NwT1Hv396CP6PXGWRNv29vbpbkDf9Cvrt89\/hzL5eXlrwi7fsWiBZnGsf\/HUHf946Yvsex\/m47lZN9VhcVkTh5FgAABAgQIdFDAWyEdPOm2TIAAAQIEZiUgLGYl67gECBAgQKCDAsKigyfdlgkQIECAwKwEhMWsZB2XAAECBAh0UEBYdPCk2zIBAgQIEJiVgLCYlazjEiBAgACBDgoIiw6edFsmQIAAAQKzEhAWs5J1XAIECBAg0EEBYdHBk27LBAgQIEBgVgLCYlayjkuAAAECBDooICw6eNJtmQABAgQIzEpAWMxK1nEJECBAgEAHBYRFB0+6LRMgQIAAgVkJCItZyTouAQIECBDooICw6OBJt2UCBAgQIDArAWExK1nHJUCAAAECHRQQFh086bZMgAABAgRmJSAsZiXruAQIECBAoIMCwqKDJ92WCRAgQIDArASExaxkHZcAAQIECHRQQFh08KTbMgECBAgQmJXA\/wF38f5ulLH5bgAAAABJRU5ErkJggg==","height":320,"width":427}}
%---
%[output:7fe7d649]
%   data: {"dataType":"error","outputData":{"errorType":"runtime","text":"Error using <a href=\"matlab:matlab.internal.language.introspective.errorDocCallback('LSQ_motor_model>AK10lsqPos\/tracklsq')\" style=\"font-weight:bold\">LSQ_motor_model>AK10lsqPos\/tracklsq<\/a>\nProgram interruption (Ctrl-C) has been detected.\n\nError in <a href=\"matlab:matlab.internal.language.introspective.errorDocCallback('lsqnonlin', 'C:\\Program Files\\MATLAB\\R2023b\\toolbox\\shared\\optimlib\\lsqnonlin.m', 242)\" style=\"font-weight:bold\">lsqnonlin<\/a> (<a href=\"matlab: opentoline('C:\\Program Files\\MATLAB\\R2023b\\toolbox\\shared\\optimlib\\lsqnonlin.m',242,0)\">line 242<\/a>)\n            initVals.F = feval(funfcn{3},xCurrent,varargin{:});\n\nError in <a href=\"matlab:matlab.internal.language.introspective.errorDocCallback('LSQ_motor_model>AK10lsqPos', 'C:\\Users\\Gavin\\Documents\\GitHub\\T-Motor-AK10-9-Modelling\\LSQ_ID\\LSQ_motor_model.mlx', 83)\" style=\"font-weight:bold\">LSQ_motor_model>AK10lsqPos<\/a> (<a href=\"matlab: opentoline('C:\\Users\\Gavin\\Documents\\GitHub\\T-Motor-AK10-9-Modelling\\LSQ_ID\\LSQ_motor_model.mlx',83,0)\">line 83<\/a>)\npid = lsqnonlin(@tracklsq,pid0,[1.0, 0.5, 0.01],[1.7, 1, 0.5], options);\n\nCaused by:\n    Failure in initial objective function evaluation. LSQNONLIN cannot continue."}}
%---
