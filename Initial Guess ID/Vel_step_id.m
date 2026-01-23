load("Data\StepTestsRound1V1.mat");
%%
M6steps = cell(1, 4);
M5steps = cell(1, 4);
M3steps = cell(1, 4);
M2steps = cell(1, 4);

cmd = cell(1,4);
for i=1:4
    M6steps{i} = timeseries2timetable(data{i+10}{10}.Values.w);
   
    % M5steps{i} = timeseries2timetable(data{1}{17}.Values);
    % M5steps{i} = M5steps{i}(18000:23000,:);
    % M3steps{i} = timeseries2timetable(data{i}{9}.Values);
    % M3steps{i} = M3steps{i}(18000:23000,:);
    % M2steps{i} = timeseries2timetable(data{i}{9}.Values);
    % M2steps{i} = M2steps{i}(18000:23000,:);
    cmd{i} = timeseries2timetable(data{i+10}{3}.Values.w);
    cmd{i} = retime(cmd{i}, 'regular', 'linear','SampleRate',1000);
    M6steps{i} = M6steps{i}.w(4000:11000,:);
    cmd{i} = cmd{i}.w(4000:11000,:);
end
%%
%[text] Data was analysed in SystemID tool/app and sessions saved as M1VelID.sid and M2VelID.sid for each respective motor
%%
plot(M6steps{1}) %[output:9308dcb4]
hold on %[output:9308dcb4]
plot(cmd{1}) %[output:9308dcb4]
hold off %[output:9308dcb4]
%%

%%
%[text] Find coefficients according to the closed loop TF:
%[text] $\\frac{\\omega}{\\omega\_{des}}=\\frac{1}{Js + {b}+1}$ 
%[text] Assuming unity feedback
%%
tf1 = M6_tf %[output:3a2a36a4]
[num, den] = tfdata(tf1);
den1 = den{1}/num{1}(2) %[output:59751fbd]
J1 = den1(1) %Kg*m^2 %[output:24d2de60]
b1 = den1(2)-1 % Nms/rad %[output:6952a3e6]
% [num, den] = tfdata(tf2); %[output:5b2b5667]
% den2 = den{1}/num{1}(2)
% J2 = den2(1) %Kg*m^2
% b2 = den2(2)-1 % Nms/rad
%[text] Measured inertias and dampings are:
%[text] M1:
%[text] J = 0.0188 Kgm^2
%[text] b = 0.0695 Nms/rad
%[text] M2:
%[text] J = 0.0186 Kgm^2
%[text] b = 0.0483 Nms/rad
%[text] 

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":30.3}
%---
%[output:9308dcb4]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAW8AAADdCAYAAAB0SfPeAAAAAXNSR0IArs4c6QAAGglJREFUeF7tnVFoXFd6gE8crN1uYxzcEDGixIiI6imBWAQFSl29FQZVD0IVJaVErBSwESpkUdHDChmngmI6NA9i6hBLYL0IKug+qCZkW5ZOlUKipB7K7kMJCIS3QbPyS9crbbqr4KT8Rz7jM6MZSSP\/4\/vfme+CsWbm3v+e8\/3nfvPrzNWcZy5cuPCtY4MABCAAgVQReAZ5pypfNBYCEICAJ4C8GQgQgAAEUkgAeacwaTQZAhCAAPJmDEAAAhBIIQHkncKk0WQIQAACyJsxAAEIQCCFBJB3CpNGkyEAAQggb8YABCAAgRQSQN4pTBpNhgAEIIC8GQMQgAAEUkgAeacwaTQZAhCAQOLyvn37tuvt7fWZWF9fdzMzM\/7n0dFRNzk56To6Ovzjvb09Nzs76zY2NsgaBCAAgbYnkKi8p6amXF9fnxsbG3P9\/f1ufn7era2tuYWFBSevDQwMuOnpabe1tdX2iQIABCAAgZhAovKuToVU4Ts7O776vnHjhuvs7PRiZ4MABCAAgUoCZuRdXXnH0ynS5JWVFV+Rs0EAAhCAgJFvFZQq+\/Lly257e9tPk8iWy+Xc5uamr8JlCmVkZMTl83m3urpakbdMJkMeIQABCDSdQKlUavo5GjmBmcpbGi0S7+npOTTP3d3dXSHz0EERt3yIeenSpUb6zL4QgAAEGiZQLBb953JWJG5K3lJhDw0NHbqrJMi7UChUTJ2ItKUatwS04RHRhANee+01NzExAZcabGFTf8DB5ng2w8PDyDtU2vGHkjLPLdvNmzfd3NycW15e9tMk9aQe5G0JaBNc3HBI+Y0km826paWlho9t9QNgUz\/DsKnPxqJrEq+84w8mw5y33BoY3+e9v79fc77bItBWlx\/9g0A7ErDomsTl\/SQDwSLQJ+kPx0IAAjYJWHQN8rY5VmgVBCBgiADyVk6GRaDKXSQcBCBggIBF11B5GxgYNAECELBNAHkr58ciUOUuEg4CEDBAwKJrqLwNDAyaAAEI2CaAvJXzYxGochcJBwEIGCBg0TVU3gYGBk2AAARsE0DeyvmxCFS5i4SDAAQMELDoGipvAwODJkAAArYJIG\/l\/FgEqtxFwkEAAgYIWHQNlbeBgUETIAAB2wSQt3J+LAJV7iLhIAABAwQsuobK28DAoAkQgIBtAshbOT8WgSp3kXAQgIABAhZdQ+WtODB+\/9yzitEIBQEIfLn70AQE5K2cBktARdw\/uPS77s\/+4HeUe0k4CLQvgb+\/u+feK\/46cQCWXBNgJF55xyvprK+v+9XiwxZeS8NKOiLvT\/78BfeDf\/+Vs1ItJD7iaQAEnpDA\/+w9NHE9Ie+qRMralH19fW5sbMz19\/f7BXPX1tb8IsPxSvKDg4M1Fya2BDTIe\/TO\/7pPSvtPOGQ5HAIQsETAkmvMVN5xgqTS3tnZ8dV3\/HO12MMxloAib0uXGm2BgC4BS64xJ+9Y0Hfu3HG5XM4VCgVfhXd3d\/vHm5ubFdMqAejk5KQrFou62WowGvJuEBi7QyAlBDKZjJN\/+XzeDQ8Pu1KpZKLlic95CwWZIrl8+bILq8e\/+OKLFVMox8lbYiwuLrqlpaXEoCLvxNBzYgg0lcD4+LibmJjw50DedVCHee733nvPvfPOOyeuvGWuXCrvJN8RkXdTrx+CQyAxAlJ1Z7NZL3DkXScN8gHm0NCQm52ddVevXi3PfzPnndi45cQQgIBzjjnvqmEglXZnZ6e\/20Q2+ZBSNnnM3SZcMxCAgBUCyLtGJuL7vMOc99bWVlnmvb29Lk33eXOroJXLjXZAQI8A8tZj6SNZAsqct3JyCQcBQwQsuSZgMXG3yWlzZAko8j5tFjkOAvYJWHIN8lYeL7G8N776PffNAxv3gip3k3AQaEsCyFs57ZaABnm\/\/fzfuv88+4rbvXO93Nuv7911z57v8o\/PPJ9x3\/zyQOzys7z23VcH3df3Hv+RUXj+7MU+9+z5jN\/34YOSP+7hg20nz4cYZy9eKh8bvyYx4k3OE44LbYjbEtoj\/0vs8FqIGWLFbY\/3C+cLbZT+ynPSfulbdb8lXvxazEH69NXHt9x3Xh2sYBW3Ie5faFN4PZwrsKk+d3W7q\/MT4sTtCz8HTr\/96R139qU+36\/QlzhPMeNwrLRH8hi2Wrn\/+ud3fb\/jfSRvsu\/BebpcHCe0Q16rjh+36zfS3kfjJoyvEEvyE46N8xvaIP2UdsVbrf1irnF7qsdM3M4wvqvHf\/x89bkrGvKUHlhyTegy0yZKyY\/l\/R\/\/fa8cVS6YsP32p\/\/sL3i5gGMpy8\/hud\/87I47N3itfLFWN0\/2O5B95cUk+8kgP3O+y8euPsfBhbvtw8WvnXn+4E0lvHYgjz\/1zx1I7fGbh7z23VcG\/Xlkn68+\/sC\/fiCxrnIMOddjWWz712ST\/kv7pP1ybDhPOH\/cllrtj9sU9j0Q411\/Dokvxwk\/efOMOcZsZF855psH274N0nd5LsQKjKQfIWchD0G+cqxsgXe9YST7x2Mg7qucV9ol\/MIW2hkeV7cxMDiQ4+P2xexCfiR+4BjaHeczjlGrH3FfQxvjN58wzsLYk32qx3VoVygeAs8wXnxR8oilZ\/UoRhjnkkd5g0p6Q97KGbAENJb3+qef+MoxbGfOZ041jXLa406D+Wme6zTt45gDAuQpmZFgyTWBAJW30lg4St5KpyAMBCCQEAHkrQzeElDkrZxcwkHAEAFLrqHyVh4Ysbz\/7cc\/MjFPp9xFwkGgbQkgb+XUWwKKvJWTSzgIGCJgyTVU3soDA3krAyUcBAwRQN7KybAEFHkrJ5dwEDBEwJJrqLyVBwbyVgZKOAgYIoC8lZNhCWgs75\/8Y\/7QX6Qpd51wEIDAUyRgyTVU3sqJR97KQAkHAUMEkLdyMiwBRd7KySUcBAwRsOQaE5V3WFi4q+vguy\/ixRhGR0edrArf0dHhX9vb2\/PLo21sbJRTagko8jZ0pdEUCCgTsOQaE\/KWpc5km5mZcdUrxMt6lgMDA256etqFlXWq82EJKPJWvloIBwFDBCy5xoS8q3MTr1t55cqVivUta+XRElDkbehKoykQUCZgyTUm5R0vQByvbSmNXVlZcQsLCxUpsQQUeStfLYSDgCECllxjTt4yTTI0NOTnte\/fv+9yuZzb3Nz0Uyry2sjIiMvn8251dfXQnLfMjReLjxczSCLnyDsJ6pwTAs0nkMlknPwT\/wwPD7tSycYqWSa+EraenENaqufDw\/Ph3VAeLy4uuqWlpeZnss4ZkHdi6DkxBJpKYHx83E1MTPhzIO8Itcxzi4Sr7ySJsxHkXSgUKqZOgrzn5+d95Z3kOyLybur1Q3AIJEZAqu5sNusFjrwfpSGeKolvAezv73dzc3NueXnZT5PU28\/SPBTyTuza4sQQaDoBS64xMedd\/aGkNCrc6\/3666+X7\/Pe398\/NN8t+1oCGsv7Xz744amWPWv6COQEEIDAqQhYco0JeZ+KYnSQJaDI+0mzyfEQsEvAkmuQt\/I4Qd7KQAkHAUMEkLdyMiwBRd7KySUcBAwRsOQaKm\/lgYG8lYESDgKGCCBv5WRYAoq8lZNLOAgYImDJNVTeygMDeSsDJRwEDBFA3srJsAQUeSsnl3AQMETAkmuovJUHBvJWBko4CBgigLyVk2EJKPJWTi7hIGCIgCXXUHkrDwzkrQyUcBAwRAB5KyfDElDkrZxcwkHAEAFLrqHyVh4YyFsZKOEgYIgA8lZOhiWgyFs5uYSDgCECllxD5a08MJC3MlDCQcAQAeStnAxLQJG3cnIJBwFDBCy5hspbeWAgb2WghIOAIQLIWzkZloAib+XkEg4ChghYco2JyjusTdnV1eXbE1bR2dra8o\/DSjuspGNoFNMUCLQhAeRdlXRZfFi2mZkZV71CvLzW09Pjpqen3eDgoBsaGjq0SLEloHHl\/dHffb8NhzddhkDrErDkGhOVd3WqY2Ffu3bN7ezseLHLgsSyQvza2lrN1eMtrOiMvFv3wqVnEEDex4wBmSaR7fr16y6Xy7lCoeBlXV2VhzCWgCJvLnAItC4BS64xV3lPTU2Vp0akcXGlfZy8JycnXbFYTHTkIO9E8XNyCDSNQCaTcfIvn887C7\/lm5K3iHtkZMTDWV1dLVfaJ628pTOLi4tuaWmpaQk8LjDyPo4Qr0MgnQTGx8fdxMSEbzzyjnIo89zyK8ns7Kzb2NgovyJTKCed85YqXSrvUqmU2OhA3omh58QQaCoBqbqz2awXOPJ+hDqeKonFLS9zt0lTxyPBIQCBBggw510FK9zHHT8d3+ud1vu8uVWwgauCXSGQAgLIWzlJloAybaKcXMJBwBABS64JWJ65cOHCt4YYNdQUS0CRd0OpY2cIpIqAJdcgb+Whg7yVgRIOAoYIIG\/lZFgCiryVk0s4CBgiYMk1VN7KAwN5KwMlHAQMEUDeysmwBBR5KyeXcBAwRMCSa6i8lQcG8lYGSjgIGCKAvJWTYQko8lZOLuEgYIiAJddQeSsPDOStDJRwEDBEAHkrJ8MSUOStnFzCQcAQAUuuofJWHhjIWxko4SBgiADyVk6GJaDIWzm5hIOAIQKWXEPlrTwwkLcyUMJBwBAB5K2cDEtAkbdycgkHAUMELLmGylt5YCBvZaCEg4AhAshbORmWgCJv5eQSDgKGCFhyDZW38sAI8v6++yv3r7d+qBydcBCAQJIEkPcR9OM1K2W30dFRJ6vCd3R0+KP29vYOrXNpCSjyTvLS4twQaC4BS64xVXmH5c7W19fdzMyMb5usbzkwMOCmp6fd1tZWzcxYAoq8m3vxEB0CSRKw5BoT8u7u7na5XM7t7u769oTV4uVnWYC4s7PTjY2N1c2ZJaDIO8lLi3NDoLkELLnGhLxj3NXTJtWLE6+srLiFhYWKDAWgMr1SLBabm71joiPvRPFzcgg0jUAmk3HyL5\/Pu+HhYVcqlZp2rkYCm1nDMpZ3qMg3Nzf9NIpMoYyMjHh4q6ur5f4FecsTi4uLbmlpqZG+q+6LvFVxEgwCZgiMj4+7iYkJ3x7kXSMt1ZV3vEu1zMNrQd7z8\/O+8k7yHRF5m7nWaAgEVAlI1Z3NZr3Akfcp5V0oFCqmTizNQyFv1euFYBAwRcCSawIYk9Mm\/f39bm5uzi0vL\/tpEpk2GRoa4lZBU8OZxkCgfQgg7yNyfdR93vv7+4fmuyWUJaBU3u1zIdPT9iNgyTXmKu\/TDAdLQJH3aTLIMRBIBwFLrkHeymMGeSsDJRwEDBFA3srJsAQUeSsnl3AQMETAkmuovJUHBvJWBko4CBgigLyVk2EJKPJWTi7hIGCIgCXXUHkrDwzkrQyUcBAwRAB5KyfDElDkrZxcwkHAEAFLrqHyVh4YyFsZKOEgYIgA8lZOhiWgyFs5uYSDgCECllxD5a08MJC3MlDCQcAQAeStnAxLQJG3cnIJBwFDBCy5hspbeWAgb2WghIOAIQLIWzkZloAib+XkEg4ChghYcg2Vt\/LAQN7KQAkHAUMEkLdyMiwBRd7KySUcBAwRsOQaKm\/lgRHkPfZgxP1kNa8cnXAQgECSBJC3Mn1LQJG3cnIJBwFDBCy5xlzlXWsBYnmut7fXpWklHSpvQ1ccTYGAEgHkXQdkkPT6+rqbmZnxe924ccP19PS46elpNzg4mJo1LJG30tVCGAgYIoC8q5LR3d3tcrmc293d9a\/s7OyU5R1X4rIg8fz8vFtbWzO\/ejzyNnTF0RQIKBFA3keAjGUdpF4oFLysw+PNzc2y3CVUADo5OemKxaJSmk4Xhjnv03HjKAhYJ5DJZJz8y+fzbnh42JVKJRNNfubChQvfWmjJUZX2cfKW9i8uLrqlpaXEuoK8E0PPiSHQVALj4+NuYmLCnwN510D9JJW3TKlI5Z3kOyLybur1Q3AIJEZAqu5sNusFjryPkbe8zJx3YmOVE0MAAlUEmPM+4Zy37MbdJlw\/EICAFQLIuwF5h+qb+7ytDF\/aAYH2JYC8lXNvCShz3srJJRwEDBGw5JqAxczdJqfJkyWgyPs0GeQYCKSDgCXXIG\/lMYO8lYESDgKGCCBv5WRYAoq8lZNLOAgYImDJNVTeygMDeSsDJRwEDBFA3srJsAQUeSsnl3AQMETAkmuovJUHBvJWBko4CBgigLyVk2EJKPJWTi7hIGCIgCXXUHkrDwzkrQyUcBAwRAB5KyfDElDkrZxcwkHAEAFLrqHyVh4YyFsZKOEgYIgA8lZOhiWgyFs5uYSDgCECllxD5a08MJC3MlDCQcAQAeStnAxLQIO8\/\/Lnf+gKP\/6Rck8JBwEIJEnAkmuovJVHAvJWBko4CBgigLyVk2EJKPJWTi7hIGCIgCXXmK+8R0dHnawK39HR4du6t7fnZmdn3cbGRjmlloAib0NXGk2BgDIBS64xL++pqSk3MDDgpqen3dbWVs1UWAKKvJWvFsJBwBABS64xL29Zw7Kzs9ONjY3VTaEloMjb0JVGUyCgTMCSa8zLW1aPl\/Urw7aysuIWFhYqUmIJKPJWvloIBwFDBCy5xrS8u7u7XS6Xc5ubm25mZsbJFMrIyIjL5\/NudXX10Jy3zI0Xi8VEU428E8XPySHQNAKZTMbJP\/HP8PCwK5VKTTtXI4FTsYZltcxDB8O7oTxeXFx0S0tLjfRddV\/krYqTYBAwQ2B8fNxNTEz49iDvBtMS5F0oFCqmToK85+fnfeWd5Dsi8m4wqewOgZQQkKo7m816gSPvY5LW39\/v5ubm3PLysp8mkWmToaEhbhVMyWCnmRBoNQLMeTeQ0fg+7\/39\/UPz3RLKElAq7waSy64QSBkBS64J6FIx510vz5aAIu+UXY00FwINELDkGuTdQOJOsivyPgkl9oFAOgkgb+W8WQKKvJWTSzgIGCJgyTVU3soDA3krAyUcBAwRQN7KybAEFHkrJ5dwEDBEwJJrqLyVBwbyVgZKOAgYIoC8lZNhCSjyVk4u4SBgiIAl11B5Kw+MIO+\/+Fm3W\/\/0U+XohIMABJIkgLyV6VsCiryVk0s4CBgiYMk1VN7KAwN5KwMlHAQMEUDeysmwBBR5KyeXcBAwRMCSa6i8lQcG8lYGSjgIGCKAvJWTYQko8lZOLuEgYIiAJddQeSsPDOStDJRwEDBEAHkrJ8MSUOStnFzCQcAQAUuuofJWHhjIWxko4SBgiADyVk6GAP2nP\/7a\/cLIgqBnnu9yf\/1f3+OPdJTzTDgIJE0AeTeYgdu3b7ve3l531Eo6Y39y2ZVKv3BnzmfK0b95UKp4HF6o9Xy9fY9qar1j7na84tY\/\/cR99fGtBnvK7hCAgGUCyLuB7Ny4ccP19PS46elpNzg4WHMNy++8OujODV4rR334oOSejSQeXqj1fHiu3jFHHSuvxcfFP3\/18QfIu4E8sysE0kAAeTeQJam6d3Z23MzMjJMFiWWF+LW1tUOrx1959x\/c\/N\/Mu53\/exz8m1+W3NmLl8qCFbnKdvalvvJOIlnZpIr+3h+97b6+V3QPH2y7Z893+eflOdlkKkT2lZjh+bMX+9zunev+OTlGHof4ft9H52ugu6q7htWuP\/zwQ1cyMqWk2sEnCAab+vBgU58N8j7hRdfd3e1yuZwrFApe1uHx5uaml3nYAtDJycmnJimZnqkn56NeO2HXVXaTizCfz7unyUWl4U8hCGyOljfjpjafMG6Gh4efmmuOuxxMLkBcXWnXk7cAnZ2d9avIs0EAAhBoJoFisegLIiubSXmftPIWiCJw+ccGAQhAoJkEZArS0jSkSXlLAk4y593MRBEbAhCAgGUCZuV9krtNLIOlbRCAAASaScCsvEP1fdR93s0EQ2wIQAAClgmYlrdlcLQNAhCAQJIEkHeS9Dk3BCAAgVMSSK28R0dH\/W07HR0d7osvvnBjY2OnRJCOw8IdOF1dB39EtL297f\/6dGtryz+u91UC4bbL55577tAxrchQ+vT222+7W7duudXVVdg45+I87+3t+dtrNzY2YOOcm5qacm+++aZnUe0R69dUKuUd3wd+584d\/wc91X\/Akw4ln7yV8gGubPJHStX3vdf7cPf+\/fsVbGQwyiZvdK3IMHB54YUX\/B8pibzbnU3130wIj87OTj8G2p1N\/Eb\/+eefV1wraWCTSnkL9Lfeesu9++67voKIQYdK9ORaTOeecZ+vXbtW86sEPvvsMzc3N+eWl5e9yKTKGBoa8pXXxYsXW46hMHnppZecyDtU3vVuOW0XNpLzvr6+mr+Zwubx9SAeiYubNLBJpbxlQA4MDJSnDWIphV8H06nkk7c6DLTr16\/X\/SqBu3fvVgg6rjSk+molhuEN\/aOPPvJvUCLvUE3V+pqFdmEjb2jnz593L7\/8spOpszBtEn4ra2c29X77fP\/991NxTaVS3tWVdrvJO+6v6D7+0q54SuXLL7+s+DbGWN5SjYVvbZTfVtLOUN7MRMjyZWZhzvvevXttz0aulTfeeKM8jRTe9G\/evNn2bOTaiT8TWllZ8d+ldNTXc1i6plIp73auvKXvIyMj5YvxqK8SaJfqMp4aqDWP2c7VZTzHLbIKb9IiKZl6bGc21QVL2n6bTaW823XOWy5E+RKu+G6BcKdJra\/PbZd53XBXQDzpFBbwyGazbf15wFGFztWrV9uaTTzHHb+xyfWVBjaplHcr3ilx3Gz3UdMaafhk\/Lj+ab1efatgu7ORayV8aB0+A9jd3eVuk+i3kFAMiczPnTvnP0u7cuVKzcVgLN3BlUp5y4XeivcoHyWwWtVlfK+39XtSteR8XBzu8z5MKL5W+PuASj7y5n758mX\/ZNrugU+tvI+7iHkdAhCAQCsTQN6tnF36BgEItCwB5N2yqaVjEIBAKxNA3q2cXfoGAQi0LAHk3bKppWMQgEArE0DerZxd+gYBCLQsAeTdsqmlYxCAQCsTQN6tnF36BgEItCwB5N2yqaVjEIBAKxP4fwyBtwfAVr3cAAAAAElFTkSuQmCC","height":0,"width":0}}
%---
%[output:3a2a36a4]
%   data: {"dataType":"text","outputData":{"text":"\ntf1 =\n \n  From input \"u1\" to output \"y1\":\n    47.51\n  ---------\n  s + 51.95\n \nName: M6_tf\nContinuous-time identified transfer function.\n\nParameterization:\n   Number of poles: 1   Number of zeros: 0\n   Number of free coefficients: 2\n   Use \"tfdata\", \"getpvec\", \"getcov\" for parameters and their uncertainties.\n\nStatus:                                             \nEstimated using TFEST on time domain data \"M6Step1\".\nFit to estimation data: 95.92% (stability enforced) \nFPE: 0.1578, MSE: 0.1576                            \n \n","truncated":false}}
%---
%[output:59751fbd]
%   data: {"dataType":"matrix","outputData":{"columns":2,"name":"den1","rows":1,"type":"double","value":[["0.0210","1.0933"]]}}
%---
%[output:24d2de60]
%   data: {"dataType":"textualVariable","outputData":{"name":"J1","value":"0.0210"}}
%---
%[output:6952a3e6]
%   data: {"dataType":"textualVariable","outputData":{"name":"b1","value":"0.0933"}}
%---
%[output:5b2b5667]
%   data: {"dataType":"error","outputData":{"errorType":"runtime","text":"Unrecognized function or variable 'tf2'."}}
%---
