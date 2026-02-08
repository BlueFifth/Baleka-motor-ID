%[text] # Run validation steps
clear;
%%
%[text] ## Import validation data
TestsData1 = load("Data/StepTestsRound1V1.mat");
TestsData2 = load("Data/StepTestsRound2V1.mat");

MotorNum = 10; % Back left
%%
%[text] ## Show the bad step
SimTimePos = 6;
t = linspace(0, SimTimePos, SimTimePos*1000+1);
linwidth = 1.5;

temp = timeseries2timetable(TestsData1.data{3}{MotorNum}.Values.q);
badstep11 = temp.q(5001:5001 + SimTimePos*1000);
temp = timeseries2timetable(TestsData2.data{3}{MotorNum}.Values.q);
badstep21 = temp.q(5001:5001 + SimTimePos*1000);
temp = timeseries2timetable(TestsData1.data{10}{MotorNum}.Values.q);
badstep12 = temp.q(5001:5001 + SimTimePos*1000);
temp = timeseries2timetable(TestsData2.data{10}{MotorNum}.Values.q);
badstep22 = temp.q(5001:5001 + SimTimePos*1000);


figure %[output:953d8b6d]
plot(t, badstep11, LineWidth=linwidth, LineStyle='--', Color='#004488') %[output:953d8b6d]
hold on %[output:953d8b6d]
plot(t, badstep21, LineWidth=linwidth, LineStyle='--', Color='#DDAA33') %[output:953d8b6d]
legend(["Motor step 1", "Motor step 2"]); %[output:953d8b6d]
title("Repeated step 3 on Motor M3"); %[output:953d8b6d]
xlabel('Time (s)') %[output:953d8b6d]
ylabel('Angle (rad)') %[output:953d8b6d]
hold off %[output:953d8b6d]
figure %[output:7a1b6e01]
plot(t, badstep12, LineWidth=linwidth, LineStyle='--', Color='#004488') %[output:7a1b6e01]
hold on %[output:7a1b6e01]
plot(t, badstep22, LineWidth=linwidth, LineStyle='--', Color='#DDAA33') %[output:7a1b6e01]
legend(["Motor step 1", "Motor step 2"]); %[output:7a1b6e01]
title("Repeated step 10 on Motor M3"); %[output:7a1b6e01]
xlabel('Time (s)') %[output:7a1b6e01]
ylabel('Angle (rad)') %[output:7a1b6e01]
hold off %[output:7a1b6e01]
%%
%[text] ## Load validation steps
SimTimePos = 6;
stepsround1 = [];
stepsround2 = [];
for i = 8:10
    temp = timeseries2timetable(TestsData1.data{i}{MotorNum}.Values.q);
    temp = temp.q(5001:5001 + SimTimePos*1000);
    stepsround1 = [stepsround1, temp];
    temp = timeseries2timetable(TestsData2.data{i}{MotorNum}.Values.q);
    temp = temp.q(5001:5001 + SimTimePos*1000);
    stepsround2 = [stepsround2, temp];
end

Velstepsround1 = [];
Velstepsround2 = [];
for i = 12:15
    temp = timeseries2timetable(TestsData1.data{i}{MotorNum}.Values.w);
    temp = temp.w(2001:2001 + SimTimePos*1000);
    Velstepsround1 = [Velstepsround1, temp];
    temp = timeseries2timetable(TestsData2.data{i}{MotorNum}.Values.w);
    temp = temp.w(2001:2001 + SimTimePos*1000);
    Velstepsround2 = [Velstepsround2, temp];
end
%%
%[text] ## Simulate simplified model

mdl = 'AK109_LSQ_simpFric';
open_system(mdl); %Load the model %[output:6618e67c] %[output:198327e0] %[output:4ecea73d] %[output:8805e27d] %[output:78078fae] %[output:549510b5] %[output:71dc92de] %[output:8e37c26c] %[output:7abee56d] %[output:0478f3b6] %[output:6a012e99] %[output:4ffed7e2]
in = Simulink.SimulationInput(mdl); % Create simulation input object
in = in.setModelParameter("StopTime",num2str(SimTimePos));
in = in.setVariable('StepTime', 0, 'Workspace',mdl);

% Starting guess: From Sys ID toolbox
% J = 1.5918; % scaled by 100
% b = 0.1399; %scaled by 10 for unity solving 

% Testing initial guess
J = 1.96; % scaled by 100
b = 0.463; %scaled by 10 for unity solving 
in = in.setVariable('J',J,'Workspace',mdl);
in = in.setVariable('b', b, 'Workspace', mdl);

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
% Position
simpos = [];
for i=8:10
      in = in.setVariable('qStepSize', Tests(1, i));% , 'Workspace', mdl);
      in = in.setVariable('wStart', Tests(2,i));%;;%'Workspace', mdl);
      in = in.setVariable('wStepSize', Tests(3,i));%,;% 'Workspace', mdl);
      in = in.setVariable('Kp', Tests(4,i));%,;'Workspace', mdl);
      in = in.setVariable('Kd', Tests(5,i));% ;%'Workspace', mdl);
      out = sim(in);
      simpos =[simpos, out.yout{1}.Values.Data] ;
end

% Velocity
in = in.setVariable('StepTime', 3, 'Workspace',mdl);
simvel = [];
for i=12:15
      in = in.setVariable('qStepSize', Tests(1, i));% , 'Workspace', mdl);
      in = in.setVariable('wStart', Tests(2,i));%;;%'Workspace', mdl);
      in = in.setVariable('wStepSize', Tests(3,i));%,;% 'Workspace', mdl);
      in = in.setVariable('Kp', Tests(4,i));%,;'Workspace', mdl);
      in = in.setVariable('Kd', Tests(5,i));% ;%'Workspace', mdl);
      out = sim(in);
      simvel =[simvel, out.yout{2}.Values.Data;] ;
end
%%


for i=1:3
    figure
    plot(t, simpos(:,i), LineWidth=linwidth, LineStyle='-', Color='#BB5566')
    hold on
    plot(t, stepsround1(:,i), LineWidth=linwidth, LineStyle='--', Color='#004488')
    plot(t, stepsround2(:,i), LineWidth=linwidth, LineStyle='--', Color='#DDAA33')
    legend(["Model", "Motor step 1", "Motor step 2" ]);
    titl = sprintf('Simplified model position step %d', i+7);
    title(titl);
    xlabel('Time (s)')
    ylabel('Angle (rad)')
    hold off
end

Starttime = 2.9;
Endtime = 3.1;
ts = 1000;
t = linspace(0, Endtime-Starttime, (Endtime-Starttime)*ts+1);
for i=1:4
    figure
    plot(t, simvel(Starttime*ts:Endtime*ts,i), LineWidth=linwidth, LineStyle='-', Color='#BB5566')
    hold on
    plot(t, Velstepsround1(Starttime*ts:Endtime*ts,i), LineWidth=linwidth, LineStyle='--', Color='#004488')
    plot(t, Velstepsround2(Starttime*ts:Endtime*ts,i), LineWidth=linwidth, LineStyle='--', Color='#DDAA33')
    legend(["Model", "Motor step 1", "Motor step 2" ]);
    titl = sprintf('Simplified model velocity step %d', i);
    title(titl);
    xlabel('Time (s)')
    ylabel('Angular velocity (rad/s)')
    hold off
end

%%
%[text] ## Full model

mdl = 'AK109_LSQ';
open_system(mdl); %Load the model
in = Simulink.SimulationInput(mdl); % Create simulation input object
in = in.setModelParameter("StopTime",num2str(SimTimePos));
in = in.setVariable('StepTime', 0, 'Workspace',mdl);

% Starting guess: From Sys ID toolbox
J = 1.5918; % scaled by 100
b = 0.000018; %scaled by 10 for unity solving 
Bt = 0.8037;
Ct = 0.2978;

in = in.setVariable('J',J,'Workspace',mdl);
in = in.setVariable('b', b, 'Workspace', mdl);
in = in.setVariable('Bt', Bt, 'Workspace', mdl);
in = in.setVariable('Ct', Ct, 'Workspace', mdl);

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
% Position
simpos = [];
for i=8:10
      in = in.setVariable('qStepSize', Tests(1, i));% , 'Workspace', mdl);
      in = in.setVariable('wStart', Tests(2,i));%;;%'Workspace', mdl);
      in = in.setVariable('wStepSize', Tests(3,i));%,;% 'Workspace', mdl);
      in = in.setVariable('Kp', Tests(4,i));%,;'Workspace', mdl);
      in = in.setVariable('Kd', Tests(5,i));% ;%'Workspace', mdl);
      out = sim(in);
      simpos =[simpos, out.yout{1}.Values.Data] ;
end

% Velocity
in = in.setVariable('StepTime', 3, 'Workspace',mdl);
simvel = [];
for i=12:15
      in = in.setVariable('qStepSize', Tests(1, i));% , 'Workspace', mdl);
      in = in.setVariable('wStart', Tests(2,i));%;;%'Workspace', mdl);
      in = in.setVariable('wStepSize', Tests(3,i));%,;% 'Workspace', mdl);
      in = in.setVariable('Kp', Tests(4,i));%,;'Workspace', mdl);
      in = in.setVariable('Kd', Tests(5,i));% ;%'Workspace', mdl);
      out = sim(in);
      simvel =[simvel, out.yout{2}.Values.Data;] ;
end

%%
linwidth = 1.5;
t = linspace(0, SimTimePos, SimTimePos*1000+1);
for i=1:3
    figure
    plot(t, simpos(:,i), LineWidth=linwidth, LineStyle='-', Color='#BB5566')
    hold on
    plot(t, stepsround1(:,i), LineWidth=linwidth, LineStyle='--', Color='#004488')
    plot(t, stepsround2(:,i), LineWidth=linwidth, LineStyle='--', Color='#DDAA33')
    legend(["Model", "Motor step 1", "Motor step 2" ]);
    titl = sprintf('Full model position step %d', i+7);
    title(titl);
    xlabel('Time (s)')
    ylabel('Angle (rad)')
    hold off
end

%%
%[text] ## velocity
Starttime = 2.9;
Endtime = 3.1;
ts = 1000;
t = linspace(0, Endtime-Starttime, (Endtime-Starttime)*ts+1);
for i=1:4
    figure
    plot(t, simvel(Starttime*ts:Endtime*ts,i), LineWidth=linwidth, LineStyle='-', Color='#BB5566')
    hold on
    plot(t, Velstepsround1(Starttime*ts:Endtime*ts,i), LineWidth=linwidth, LineStyle='--', Color='#004488')
    plot(t, Velstepsround2(Starttime*ts:Endtime*ts,i), LineWidth=linwidth, LineStyle='--', Color='#DDAA33')
    legend(["Model", "Motor step 1", "Motor step 2" ]);
    titl = sprintf('Full model velocity step %d', i);
    title(titl);
    xlabel('Time (s)')
    ylabel('Angular velocity (rad/s)')
    hold off
end


%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":22.6}
%---
%[output:953d8b6d]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAALQAAABsCAYAAADDoJhwAAAAAXNSR0IArs4c6QAAE4RJREFUeF7tXX2MVcUVPwtkwbqsZlXYZbUUoVqqDZENSIkhSkOiuGyDIVCJERJZiqIQDWKiyLp+UCFUotTwsRqkMfgRoi1BUmxC0D8KuMK2KUhpIBRYWGEBhUURRLb5XTjPebNzv2fuu++9uf\/A3ndn5pzf\/O65Z87MnCmpqKjoJHtZBAoEgRJL6ALpSauGg4AWQi9cuJBGjRrVBdJPP\/2UnnrqqcShfuyxx+j6668P1fbtt99OL774Ih0+fJimTp0aWOaJEyfS6NGjacaMGYHL+D0o4nnmzBmaN28ebdu2za9Y6N9Z57KyMjpy5AjNmTOH9u\/fT9Bp5syZVFpaSnv27PHFY8CAAdTQ0EDLli2LJWcYedww0kboESNG0Ouvv07vv\/++AywalO+FRjxCAe6MrVu3Gid01JfASy1R\/g8++CDSSxYUNpFA58+fz\/QfDMLkyZOdaoIQ+q233qLq6urYL15QeTZs2OC8cOhjGSNjhGZQ1qxZQ0uXLiURJNHqMBinT5+mfv36OSByGX4x2Pq7WRE8h68BKweLI3aG+Da71YH75eXlSgstAs1tLV++nBYvXpyRmeutra3NkCGMnioS+r0woiUVceP7u3fvpoEDBxLwUBGT60dZWOO1a9c6fcV9gnuw2PhiqTDA11fEVnwpUMfNN9\/sqMXY4P\/ArFevXs59yCUawTDyMF4yRsYILVpoNM5vFBMB9\/CJw6cKioPE69evdxQGsfCZHT58uEMO\/PbZZ59lrFVjY6Pz3ObNmzMdgM8ewBHbAuDii6WqQ25L1fGiBbrvvvsyX54DBw5kWVDRuobRU+VO+H1p5N9VeB8\/ftzBGO6Q6mvJZAARcbW3tztuA1yvffv2OS8DXDC+x+6YbKxkCy3+jXrZleN+u\/baa7OILJPTTx52CVUYaSO07EOLb6vsfuDvoUOHOqR9+OGHsz5XIlg1NTVZvwGo3r17K309bk8mtAps1PHxxx87Lwu7Jl7WkK2N7M\/KZaLqCauouvCS4sXllx\/Wki+5LVEW\/iSzdZUJKBMIX0eQdfDgwbRu3TqaMGECbdq0ie644w7n\/vbt2zOGBbLKcsEoscshEpiJx7K+8847NGbMGKU+uMk6+MkjjnFkWbQRmi0AEwoE48GM+PlhMJmAY8eOVRL6o48+ottuuy3zSedyINXbb79NDzzwgPOZhFUG8eX2QVTZLRDr2LhxI40bNy4QoeVPOxNb7rwoeorulYrUbmMR+UVVEZpf1iCExtcLfQH3ABa0qamJ6uvrHUIfPXo0y8J7Ebp\/\/\/6ZrzEHBKIQ2kseedAuYqSd0BgUsl\/Fn2+vAaLcMV4Wmjtc7iA39waAug1YmKSyj+gV5eCOhK8P3eRPcVQ93Sw09HWTP4iFDkPolStXOl8+jnjw37my0F7yyIQWMTJCaLYWbEGD+NAY1LFF9fOhVaNcVVs6fGjZGg0bNsx1hB3Eh1bpKfrQbj6\/3IlBfOgwhF60aBHNnTvX+SKKA2wTPrTKhZJdDi95RBdIHBcBIyOEhnDcMUFH\/xiEDBkyhETfG\/W4RSjEz\/vJkyepoqIia2CJjuG2MShSRUrkuGfQKIcYKZH9ax7IQnZVlMNNT9HdUEUIRP+ZnxVdIRE3mexBXA4ePAIneRCuinKIrpIYweL7og78pfYaE8iE9pPHDSMthFaOaALe1BXDDNhczh4rFj1zBvDlhrURGpa0b9++ylkl8W2SLXCxdHSx6FkQhGbCqmK4\/JnhmHGuFbbtFzYCsS00LHNra6sTOsMlD17gp86fP59Wr16dmRYvbEitdrlEIDahWXhYaRWhxQEDfpeteFVVVS71t22nBIG2tjYtkhgntCglux979+51Fg6BzJh8wayhvYobgR07djhT5HGJnSih0WWiJQeRMdPnpciK6V85PT19VTV1ZrYiiHsSSqjbhW9d2XCxxxU8\/r38b3ZZ3PQq39mtlDq79XBevqrKStrRsqNLfd0unPVo\/yeu7RJdksWr\/KX2uwv1lxB1XiQqKcncU5WHvJWVlbT93\/\/JLuv8FRy\/H9vv2i7RJRn88WdZu2IPWWp+9QuaNm0aYZ1M6gkNlwP+tRjLxHoBzI4xob0UWffSDVR1TQ8HuEErJ1DnZRC5l7qdP0VX71yhJNTF0qvo61t\/72n6vMqj4Df9x9K5a251rSPN7X9f9lPquOl3qdd\/dGmzY9hSS2gMBDHrhNkezIKpguxAOQih8dz35TfSPxdderuHzfhxgU5xf6QLR\/ugPAiicZbLIQ\/guAK\/BTRBGlI9E0aRl6f3od8MvZLWbzlDjavbXZu0g8yovZFMOZVLEYYHflI6hGYie8WRMZWsm9hhFWlePsDRp+6ZQ9R24kIX3ewg06+7c\/+7avAXlgdeWpTcc889nViTjMXXqvUCYuFXX33VIbWu\/W1hFVn+RBXV3NSLZrzSRtv\/+10XvYIMMnPfpcUrAZYDqwZ\/YXngSehc7vqOokh97dXUtP5rpU5R6iteeiWvuVv\/6Ow3x0IjbMb78FRqmtp5rFORMIPM5LvStujVPzp5kDUoRDQCuxPE1ANi2E13t3gpAveiaf1XStfCTQ6dwOjWNUh9qj1yvMQVvqdbSgjcx1qZuK4gJr4efPBBx\/2Mc0GP8ePH09NPP53lxiZiodnlkENtrJDb\/TgKc1k3BRF3Rvx52\/6e9OhCcWLAu9VCIPSUKVMIe+qYDDAod999N+3cuVNJaF05MYCsjhwj8jp4cVyWKKGhEMeLOUGM1yq6JAg94c830YF\/bAzclAowr0kRVNzzxE7X+v3K+pXH7351iO2DUNjX17NnT\/rwww+dxVzYxYMlt2fPnu2yRxIDdCTUwaJ8bDpYsmQJPf7441npILCjA3MC2CfI280gl7idDH9\/\/vnnTjl5xwpcUXY5sXlh5MiRzjPYISQnEmILj\/QJObfQ3KtiLFpeuxyYWQEfdHtjJ42+iuZMrKBxDSfoy6OnXWuDW4KIh5fFPzl0rqc0FTsWuf7uVxYFvcrjd786xPJsIU+dOuWsYAQZEYECQbAjBxevOecvJ1YxYoYN+xvxr\/w7ZmXr6uoyk1ysrMoai\/dgzLDVCTO67ELs2rXL2biM3SR9+vTJmjwTQUyFyxGQg1ofcyM0h+f8ZgU5Ls3PqerD1LXXdeWBDa4\/+5VFQa\/y+N2vDrE8EwrEwXIBEIovWGJcIDqnEliwYAF98sknzo5sJrT8O3zvW265RRmWFWdwYW3RHtKaYUmwmEQH7cJKNzc3U\/fu3TOuD74eSHfA2bLElyUVFtotR13SUQ4mtNsECgP315duoH6X13ng3pFzP6c+wxu1rAnQ+uYGrIwJvWrVKsf6dXR0OETFp54JHcVC33XXXZ7zDGztt2zZQoMGDXIS04gWWvx633nnnbEt9Mz6ifTc\/dmgVFZV0W8xYRZzGWkmyiEuxIcfx58bVeQjYP\/4PuZmoXlBkp+FRgMiqTvLBtMPA5\/Ne0IzoaAfFnVx8kk5z4icbQqWG4NKMaUauy3yxJmca4QtNDJcwdfmFA0czoWP3tLS4lh79qHdZo79XA4QesWjlxaciVdd43d6Cc0LilS+GC808mVpiAfcCC27EkGrzPcoR1A9c\/VclKyuoqyJRjnExfewzsiag+w58OUgiImUrl6Ebu24ksY\/6R6BUHWqJbRZqucVoQGFGHOGleZcFjoWJak2y7oREOs1rut\/I\/3t71+E6iFL6FBwJf5wohbapHZiQhfx5dBNQN31mcSkGOtOlNCmZgR5Jgvpu+DG8G4VdKhuAuquL2nS5fvUtzxZI3\/ZEyU0Os9kRIOttIrQb7zxBr355pux+VMIhM7nqW+EfXFhbYnKQLr1z0MPPaR\/T6GcoV1kl444tBehESrCBEDcqxAInc9T32L\/qdaYuPXPvffe6wQdUrWn0I+MXoTWoYibC9Mw5TpP0by2c\/mVRcVe5fG7Xx1i+UKZ+nbLlpWIy4H10MhkP3v2bD9OUpwdK7kiNMe03ZTzmrzxK4s6\/SZ\/\/OoQyxfC1LdXpqxECJ3UnsKghMa0Ny5x0ZHvm3b5ARVgtb++dICQ24VNt26XX1mU8yqP3\/3qEMvn+9Q35McY4Pnnn1euzU6M0Nyhadj1zRatZtZJQs6LMFch+NB85qGYkCdfpr7lIznksVfihA5DHh3PqhQEoTFLiIUqxUZoHZiarCPvZgpNgqGq24vQYae93QaFSetUyO1ZQvv0rhuhm\/9XSo+8vCc0N\/Ld5QitcJ4VKDqXg\/cSIudGnEEh4tpx19XmGVfyQlwkAlLlsNNpiLRlH42CqKwIkituf62C1m3vRi807Qtdpc2cFBqyxAtgAg0GR7yMEVqci8eCbpwVN2vWLGPZ92VF+lZdQ+sbymn8ikpqbdkSCWwn7a1Noh4JuyQK4cspfz2NEJrJjG0\/WEjEe8KwewLbcrAx0i9VWFhAZEVG\/PIKWjqrkuqe\/ZLa2t1zLodtxz6fbgSMEFpcTILjbZnQXrt748KkU5G4stjyuUNAJw+6ZE7Cwe44ZxuHjCNQDsvsdVxwHBh0KhJHDls2twjo5EGXQaE8WygnE9Gpuk5FdMpl60oWAZ08SFWUI1kYbWtpQUAroQsp+2haOsjKEQ4BrYSOmx9aPgBeFQ1xOxpZpyLhILRPpwkBnTyI7XLwti1OgiIfgex1NLJORdLUQVaWcAjo5EFW5iSvxOeqxI3yGmfVAfZhFnwjYxLWcUSZJQwHoX06TQgYITQUVBGS7yH5DOc14wkWmax4Vp6E8ToaWVTk8IlvnWnvzTt\/oCf\/dDBNeFtZDCNghNB+Cc\/fe+89mjRpUlZa1iCEFrGQj0ZmRbDre8NfVjtJzqe+O4B2bd5kGEJbfZoQMLLrm8mGxNhYEYUUqZwn4vjx406uYmTBFAd9QVwOGTjV0chYrNJxbCetmVdN01\/7mlq+uHQcsr2KAwGju75lFwHJQkDm+fPnKxcp+Q0Kgx6NPG3M91Q3sjfVNp6mo20niqMnrZYOAkZcjqjYimE7PrgzytHIDfeTcwZhlL2EUWW35dKBgDFCu22S1ZFoRgWdqAgT2i8tQDq6wEqhEwEjhFalGdAptB+hkQAbO1ashTaNevrqN0ZoTnge97y7oJCJiqxr6OUUsxY6KHqF85wRQgMeVRzaJGyiIhfpPA372TnfxC0m5bF15wYBI4Q2nazRz+Wwm1pzQ6Y0tGqE0LlQTKciuZDftqkHAZ08iL04KY5KOhWJI4ctm1sEdPLAk9A8U4iFSUkeGpRbeG3rSSNglNCq8+uQkd3EpVMRE\/LZOpNBQCcPHAstkhj5OLBeo6GhIXP4pim1dCpiSkZbr3kEdPKgBFuwVPFn1dG4ulVjRXr8azK1nbjgrOMIm3FUt0y2vuQR0EpoWGg5ZIed3jhTmo9HNqWiSOgoB22aksvWmywC2gktiy8fYq\/j4E2vODQsdNSMo8lCb1szgYBxQotCg9xo0FiUY8lz1GP3bHp3Wyn9cVX4FLomALZ1JotAooTWoZq4ik9MXANFljXWUbfDq6n2hW\/o6OFjOpqzdeQZAnlFaHGbFnAWD5WBIsufqKSSM7up7plDzsDQXsWHQF4RGta5rq7OcVmOHTtGixcvJk51AEVWTP\/KOVPlkVfSv0sFh2K2tLTkTTL1fJHXLRF6lFfb+NQ3CM27xSEgCL13717n+FwogmWjref6U+XwP0SR35YpEARUidCjqJZTQkNgm6A8SrcVXhlVIvQoWiZCaDeXI4rAtoxFwAsB44T2GhTarrEI6EbAOKEhsBi2MzVJoxsYW19+IpAIoVXQuMWm0wijeJgS5ON0DWmUVZRJlZotjTKrFsdFPc8nJ4TONzdE3GvJ614wKje1rFYH6cSsVyYOfNIhI+qQ08nFXRSXE0J7xaZ1AWWyHs4WlVZC44uyYMECam9vp+rqaiMnmOnCV8yspaPOnBHaLTatQymTdXilBzbZbpi68UXB1dra2iVjbJh6kniW5ynKy8uprKyMeD1+XrkcXpMtSYAYtQ2v5O1R69RdDq4GH8lXW1ubekKLi994Jpkn3qJgkzMLnW+x6XywzCCAvPQX9+JavSjEClpGNG6wymJ22qB1iM\/lhND5Nih0y50dBfAky8hkSbLtoG2JXGhubs5aGhG0jpwTGgLkU2xaPPSIwTN5fmOUjlSVyQdCy1yIGxLNiYXW1WG2HouAjIAltOVEQSFgCV1Q3WmVsYQOyQF5GlwsjmgCskwdPHjQyCxikMFp3Jm2kHCk7nFL6BhdgphvfX09NTU1OYcsmb6CkDUI6U3Lmcv6LaFjoK8iNE+LI6cJ9k8eOnSIhgwZ4ljutWvXOtvRMCMmjubFiI9bzFicMEG8VlzQIx+KyjOFaZ2ajwG5b1FLaF+I3B\/wIzSOq9u6davjfoDocFdwZB4utuzi\/zkO29HRQVOnTs1qWCSpPGMpr4eQyR9Dxbwragkdo8v8CC26I\/KKPT4mr6amJuv0XbfYsehu+E3BF7PbYQltkNBiygYvQo8aNSpLCvnUMRWBxfRt8vOW0DE6tZiL+lnooIRGHkHZxZBx9RoQylbdErqYWRlDdx2EFn1oREpA3N69e3dZwyz60PJCKetD\/9iJ1uXIMaFBYjHK4XbIqTzQ8ypjoxwxOtUWTQ4BG4f2x9paaH+MUvNEEN84COlTo5ABQSyhDYBqq8wdAv8HMOf3\/9aTEOcAAAAASUVORK5CYII=","height":108,"width":180}}
%---
%[output:7a1b6e01]
%   data: {"dataType":"image","outputData":{"height":108,"width":180}}
%---
%[output:6618e67c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Error evaluating <a href=\"matlab:Simulink.internal.OpenCallbackParamsDialog(['sm_lib\/Body Elements\/File Solid'],'DeleteFcn');\">'DeleteFcn'<\/a> callback of File Solid block (mask) '<a href=\"matlab:open_and_hilite_hyperlink ('sm_lib\/Body Elements\/File Solid','error')\">sm_lib\/Body Elements\/File Solid<\/a>'. \nCallback string is 'simmechanics.sli.internal.block_dialog(gcbh,'close');\nsimmechanics.sli.internal.rtm_callback('DeleteFcn',gcbh)'"}}
%---
%[output:198327e0]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Invalid feature 'SMPIDialogs'."}}
%---
%[output:4ecea73d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Error evaluating <a href=\"matlab:Simulink.internal.OpenCallbackParamsDialog(['sm_lib\/Frames and Transforms\/Rigid Transform'],'DeleteFcn');\">'DeleteFcn'<\/a> callback of Rigid\nTransform block (mask) '<a href=\"matlab:open_and_hilite_hyperlink ('sm_lib\/Frames and Transforms\/Rigid Transform','error')\">sm_lib\/Frames and Transforms\/Rigid Transform<\/a>'. \nCallback string is 'simmechanics.sli.internal.block_dialog(gcbh,'close');\nsimmechanics.sli.internal.rtm_callback('DeleteFcn',gcbh)'"}}
%---
%[output:8805e27d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Invalid feature 'SMPIDialogs'."}}
%---
%[output:78078fae]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Error evaluating <a href=\"matlab:Simulink.internal.OpenCallbackParamsDialog(['sm_lib\/Frames and Transforms\/World Frame'],'DeleteFcn');\">'DeleteFcn'<\/a> callback of World Frame block (mask) '<a href=\"matlab:open_and_hilite_hyperlink ('sm_lib\/Frames and Transforms\/World Frame','error')\">sm_lib\/Frames and Transforms\/World Frame<\/a>'. \nCallback string is 'simmechanics.sli.internal.block_dialog(gcbh,'close');\nsimmechanics.sli.internal.rtm_callback('DeleteFcn',gcbh)'"}}
%---
%[output:549510b5]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Invalid feature 'SMPIDialogs'."}}
%---
%[output:71dc92de]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Error evaluating <a href=\"matlab:Simulink.internal.OpenCallbackParamsDialog(['sm_lib\/Joints\/Revolute Joint'],'DeleteFcn');\">'DeleteFcn'<\/a> callback of Revolute Joint block (mask) '<a href=\"matlab:open_and_hilite_hyperlink ('sm_lib\/Joints\/Revolute Joint','error')\">sm_lib\/Joints\/Revolute Joint<\/a>'. \nCallback string is 'simmechanics.sli.internal.block_dialog(gcbh,'close');\nsimmechanics.sli.internal.rtm_callback('DeleteFcn',gcbh)'"}}
%---
%[output:8e37c26c]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Invalid feature 'SMPIDialogs'."}}
%---
%[output:7abee56d]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Error evaluating <a href=\"matlab:Simulink.internal.OpenCallbackParamsDialog(['sm_lib\/Utilities\/Mechanism Configuration'],'DeleteFcn');\">'DeleteFcn'<\/a> callback of Mechanism\nConfiguration block (mask) '<a href=\"matlab:open_and_hilite_hyperlink ('sm_lib\/Utilities\/Mechanism Configuration','error')\">sm_lib\/Utilities\/Mechanism Configuration<\/a>'. \nCallback string is 'simmechanics.sli.internal.block_dialog(gcbh,'close');\nsimmechanics.sli.internal.rtm_callback('DeleteFcn',gcbh)'"}}
%---
%[output:0478f3b6]
%   data: {"dataType":"warning","outputData":{"text":"Warning: Invalid feature 'SMPIDialogs'."}}
%---
%[output:6a012e99]
%   data: {"dataType":"text","outputData":{"text":"Operation terminated by user during <a href=\"matlab:matlab.lang.internal.introspective.errorDocCallback('LiveEditorEvaluationHelperEeditorB0DFBBCBmotw', 'C:\\Users\\Gavin\\AppData\\Local\\Temp\\Editor_elriz\\LiveEditorEvaluationHelperEeditorB0DFBBCBmotw.m', 71)\" style=\"font-weight:bold\">LiveEditorEvaluationHelperEeditorB0DFBBCBmotw<\/a> (<a href=\"matlab: opentoline('C:\\Users\\Gavin\\AppData\\Local\\Temp\\Editor_elriz\\LiveEditorEvaluationHelperEeditorB0DFBBCBmotw.m',71,0)\">line 71<\/a>)\n\n","truncated":false}}
%---
%[output:4ffed7e2]
%   data: {"dataType":"error","outputData":{"errorType":"runtime","text":"Program interruption has been detected."}}
%---
