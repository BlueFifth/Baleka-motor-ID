load("Data\StepTestsRound1V1.mat");
%%
M6steps = cell(1, 5);
M5steps = cell(1, 4);
M3steps = cell(1, 4);
M2steps = cell(1, 4);
S_start = 4000;
S_end = 6000;
cmd = cell(1,5);
for i=1:5
    M6steps{i} = timeseries2timetable(data{i+10}{8}.Values.w);
    M5steps{i} = timeseries2timetable(data{i+10}{9}.Values.w);
    M3steps{i} = timeseries2timetable(data{i+10}{10}.Values.w);
    M2steps{i} = timeseries2timetable(data{i+10}{7}.Values.w);

    cmd{i} = timeseries2timetable(data{i+10}{3}.Values.w);
    cmd{i} = retime(cmd{i}, 'regular', 'linear','SampleRate',1000);
    M6steps{i} = M6steps{i}.w(S_start:S_end,:);
    M5steps{i} = M5steps{i}.w(S_start:S_end,:);
    M3steps{i} = M3steps{i}.w(S_start:S_end,:);
    M2steps{i} = M2steps{i}.w(S_start:S_end,:);
    cmd{i} = cmd{i}.w(S_start:S_end,:);
end
%%
%[text] Data was analysed in SystemID tool/app and sessions saved as M1VelID.sid and M2VelID.sid for each respective motor
%%
plot(M6steps{1}) %[output:1adb5e10]
hold on %[output:1adb5e10]
plot(cmd{1}) %[output:1adb5e10]
hold off %[output:1adb5e10]
%%

%%
%[text] Find coefficients according to the closed loop TF:
%[text] $\\frac{\\omega}{\\omega\_{des}}=\\frac{1}{Js + {b}+1}$ 
%[text] Assuming unity feedback
%%
tf1 = M6_tf %[output:24248189]
[num, den] = tfdata(tf1);
den1 = den{1}/num{1}(2);
JM6 = den1(1) %Kg*m^2 %[output:4fcef7bc]
bM6 = den1(2)-1 % Nms/rad %[output:4ebc8202]

tf1 = M5_tf %[output:89866e1f]
[num, den] = tfdata(tf1);
den1 = den{1}/num{1}(2);
JM5 = den1(1) %Kg*m^2 %[output:7f9e0329]
bM5 = den1(2)-1 % Nms/rad %[output:6256d6c2]

tf1 = M2_tf %[output:8c0fffb9]
[num, den] = tfdata(tf1);
den1 = den{1}/num{1}(2);
JM2 = den1(1) %Kg*m^2 %[output:405fb475]
bM2 = den1(2)-1 % Nms/rad %[output:9dc63e61]


%[text] Measured inertias and dampings are:
%[text] M3:
%[text] J = 0.0210 Kgm^2
%[text] b = 0.0933Nms/rad
%[text] M2:
%[text] J = 0.0175 Kgm^2
%[text] b = 0.0097 Nms/rad
%[text] 
%[text] M5:
%[text] J = 0.0180 Kgm^2
%[text] b = 0.0098 Nms/rad
%[text] 
%[text] M6:
%[text] J = 0.0185 Kgm^2
%[text] b = 0.0101 Nms/rad
%[text] 

%[appendix]{"version":"1.0"}
%---
%[metadata:view]
%   data: {"layout":"onright","rightPanelPercent":30.3}
%---
%[output:1adb5e10]
%   data: {"dataType":"image","outputData":{"dataUri":"data:image\/png;base64,iVBORw0KGgoAAAANSUhEUgAAAQAAAACaCAYAAABG+Jb3AAAAAXNSR0IArs4c6QAAEW5JREFUeF7tnX9IXecZx58kxISQYuparSJxYbLBoGGJqIwSke6PjWpkk01K2qGrWgJi\/xiC6yqGOHFI89fktqHqFrdWmGuTYh10K9uypKM1a26oHV3TlYldp7N0TVwlS8ySjufV9\/b15F5\/3NxzznN8vgdC7j33vOd9nu\/3fT\/nfd9zrndTTk7Op4QNCkABlQpsAgBU+o6koYBRAABAQ4ACihUAABSbj9ShAACANgAFFCsAACg2H6lDAQAAbQAKKFYAAFBsPlKHAgAA2gAUUKwAAKDYfKQOBQAAtAEooFiBjAGgrq6OWlpaKCsri+bn56mjo4PGx8eNtL29vVRRUWFeDw8PU19f3zLJ8\/PzFVsgK\/XP07\/pO1\/cLisoxdF8\/4\/\/8TX7jABgz549dOzYMTp9+rTp3CdOnDBBNzQ0EIOhvr6eurq6qKysjCorK6mtrY0mJyfNMdz5GRb79+\/3NVGcfG0KFP70m\/S\/nbl0Y2fu2grgKF8VODDwNs3MzPhWR0YA4I2OATA7O0vt7e3m6l9cXGw6fW5uLnV2dtLQ0BCNjIyYYtzxY7EYdXd3+5qobwouQeyee+6hCxcu+FmN7+dmGB8veIOefHcHvfrJnb7X51cFnEfU\/di3bx81NTVRbW2tr\/0iowCw04CFhYXEFIABkJeXZ0YD5eXlpqOPjo4mpgEWAH4n6ldj22jn\/UdzHvGw81fv\/nejpRapfILqFxkFgFW4tbWVampqDAS4YwMA0Wl7AIAMryINAB4JNDc3U39\/P5WUlKxpCjAwMECDg4My1FccBQAgw\/zGxsboTAF4EbCnp4dOnTpl5vbuvL+0tHTFRUBLOr6DEI\/HZaivOAoAQIb5VVVViRF0JBYB13IbkNcGeMHPLgC6i4BYA5DR8AAAGT5EegqwHgmDSnQ9MWk+FgCQ4X5Q\/cKXRcD1SBhUouuJSfOxAIAM94PqFwCADL\/FRAEAyLACAJDhg7ooXABs3V1C2Q8fNxpcOfsMXZ+K0425adqSXUDX3z9P\/Dlv9rX9zBWN992cm6HN2YuPe3NZexz\/7272vPZY+xmXt3XZfd6ydj+fw3uMt0773lvGPSfXuW1vNd28\/NlTePZzW56P4Y3jtTna+u2x3txdrZI1Lnv8V76QT4e7nqIjzd+KzoNA6fSWoEiXTmway7gA2HGgmbbtPWg6\/\/Z7q01H33HgUbqx1PC3LHXqqxNjtH1vNfH\/vG\/zrgK6PnU+8Zp1vHl5mrYWlZiyfIw9xyIUFuHA+\/g4Lu9u3n32PdfB5+Ty\/JrL8We88X6O28Zr60x2flu\/jcGNZzmgFuN263fLGhjsKliWn1ufjdfus7HacvbzaxMvmUfk54oP0taf1QAAGjtiWDm7AMh+aPHqP\/fc4cSVbqWrpnvFtldx9wrolk01Clje4dY+WrCjBHfEkaxuu8\/7mfdqbuPwjkYMzJZGNMnOn0yDZDklO497Pr4w9vUsPkgXiduA6TZYjADSVc6fcl4A8FX\/ytl+fyrDWVMqEFS\/wCIgGuEyBVwA3PXDP5thNAAQfCMBAILXHDUSkRcAn4wdpWsTY9AmYAUAgIAFR3WLClgAvDC7i3JaRmnu2cNm8Q9bsAoAAMHqjdqWFAAAZDQFAECGD+qisAB48eqXzTMAGAGE0wQAgHB0V1+rFwAfx2rMbS9swSoAAASrN2rzTAHsCAAACKdpAADh6K6+Vu8I4KOeUvWahCEAABCG6qgzcRdgdPvX6I7qIwQAhNMoAIBwdFdfqx0BAADhNgUAIFz91dZuAfDrvIfNF4EuxWrUahFm4gBAmOorrhsAkGE+ACDDB3VRAAAyLAcAZPigLgoAQIblAIAMH9RFAQDIsBwAkOGDqigK79hCrz14F9WNXaK3Sh43f9nG\/jEQVUIISBYAEGCCthAAADmOAwByvFATCQAgx2oAQI4XaiIBAORYDQDI8UJNJC4A3r7\/JyZvrAGEYz8AEI7uqmv1AoD\/BPb82FHVmoSVPAAQlvKK6wUA5JgPAMjxQk0kAIAcqwEAOV6oiQQAkGM1ACDHCzWReAGAHwUJz\/rIAaC1tZUOHTpkFFtYWKBYLEYjIyPmfW9vL1VUVJjXw8PD1NfXl1A2qETDszI6NbsAeKf2eeLfqMOPgoTjX1D9IiO\/DFReXk6dnZ00NDRkOj13eE6go6ODioqKqL6+nrq6uqisrIwqKyupra2NJicnjbJBJRqOjdGqFQCQ41dQ\/SIjAPDKVldXl+j0\/OOGxcXFptPn5uYuAwUAIKfBcSQAgBw\/Ig0AHgHk5eVRQ0ODGQ3Y1zxS6O7uptHR0cQ0IKhE5VgrNxIAQI43QfWLjI8AeC3AHeYDAHIa1WqRAACrKRTc55EEgNvZrVS8by1TgIGBARocHAxOYdR0iwIAgJxG0djYSE1NTcRT6JkZ\/36YJWMjAO7ovLW3ty9T0V0PWGkRsKWlheLxuBwHFEbiAuBv3\/sT4ZeBw2sEVVVVZhE9EgDgTs4dOCsrK6HY\/Py8SWB8fDxxG9B7exCLgOE1sGQ1AwBy\/IjkFCAd+YJKNJ3YtJUBAOQ4HlS\/yNgUIF3pgko03fg0lQMA5LgdVL8AAOR4HnokAEDoFiQCAADkeKEmEguAB89up4u1z9Pcs4eJvw+ALXgFAIDgNVdfIwAgpwkAAHK8UBMJACDHagBAjhdqIgEA5FgNAMjxQk0kAIAcqwEAOV6oiQQAkGM1ACDHCzWRAAByrAYA5HihJhIAQI7VAIAcL9REYgHw0Ft76O37++jjWA3dnPPvm2hqhE0jUQAgDdFQ5PYUAABuT79MlgYAMqkmzrUmBQCANckUyEEAQCAyoxJXAQBATnsAAOR4oSYSAECO1QCAHC\/URAIAyLEaAJDjhZpILAC++\/599JeSx+mjnlI1uUtLFACQ5oiCeAAAOSYDAHK8UBMJACDHagBAjhdqIgEA5FgNAMjxQk0kAIAcqwEAOV6oiQQAkGM1ACDHCzWRuAB4s7iBLsVq1OQuLVEAQJojCuKxAGiY+zbF7\/46ABCi5wBAiOJrrRoAkOM8ACDHCzWRAAByrAYA5HihJhIAQI7VAIAcL9REAgDIsRoAkOOFmkgAADlWAwByvFATiQuAN7beS3PPHVaTu7REAQBpjiiIxwLgEXqMzl35HAAQoucAQIjia60aAJDjfGQB0NraSpWVldTW1kaTk5NG0d7eXqqoqDCvh4eHqa+vL6F0UInKsVZuJACAHG+C6hebcnJyPs1U2rajT09PJwBQV1dH9fX11NXVRWVlZbfAIahEM5XjRj4PACDH3aD6RcYAwFf+wsJCo2BxcXECAAwF+z43N5c6OztpaGiIRkZGzLFBJSrHWrmRAAByvAmqX2QMAFY6t8PzFIDf5+XlUUNDA5WXl1N3dzeNjo4mpgFBJSrHWrmRAAByvAmqX4gBwMDAAA0ODspxQGEkFgDNu35Mr00v4C5AiG2gsbGRmpqaqLa2lmZm\/Pt1pkAAsJYpQEtLC8Xj8RAlR9UuAF796xTNjx2FKCEpUFVVRR0dHdEHABYBQ2pBaVQLAKQhmk9FNswUgPWxdwcWFhYoFoslFgCxCOhT60nztABAmsL5UCyyAFivFkElut64NB4PAMhxPah+kfE1gPVKGFSi641L4\/EAgBzXg+oXAIAcz0OPBAAI3YJEAACAHC\/UROIC4Mzrr9GVs\/1qcpeWKAAgzREF8QAAckwGAOR4oSYSAECO1QCAHC\/URAIAyLEaAJDjhZpIAAA5VgMAcrxQEwkAIMdqAECOF2oiAQDkWA0AyPFCTSQAgByrAQA5XqiJxAXAH35zkq5NjKnJXVqiAIA0RxTEAwDIMRkAkOOFmkgAADlWAwByvFATCQAgx2oAQI4XaiIBAORYDQDI8UJNJC4AfvfLGF1\/\/7ya3KUlCgBIc0RBPACAHJMBADleqIkEAJBjNQAgxws1kQAAcqwGAOR4oSYSAECO1QCAHC\/URAIAyLEaAJDjhZpIXAD89pkn6Oacf79Io0bUNBMFANIUDsXSVwAASF+7TJcEADKtKM63qgIAwKoSBXYAABCY1KjIKgAAyGkLAIAcL9RE4gLg5ScfUZO3xEQBAImubPCYAAA5BgMAcrxQEwkAIMdqAECOF2oisQB4hB6jV\/qfUJO3xEQBAImubPCYAAA5BgMAcrxQEwkAIMfqDQWA3t5eqqioMOoODw9TX19fQmlOdPjBL9HEmZflqK84kq\/mZxGmAOE3gA0DgLq6Oqqvr6euri4qKyujyspKamtro8nJSaMyJ\/qLpgP0+9cnaHN2vtnHj6DeuDxDW3blp9xnj9latH+ZW9en4uTu479suzm7ILEv2bm5DG9cjj\/nOLjc1qKSxLl5n\/fc\/N6NkQ\/21mfPfXNumrbtrV72eK3N163fVujWZethTby5cYzuebgedx\/nc33q\/C37bD1c1tXy3JW76IXZO2l+7Gj4vUBxBBsGAHz1Ly4uNp0+NzeXOjs7aWhoiEZGRhIAuO+xp+nnr1wwf4HmxtwM7Tjw6CIILk+bn6jmjrN9b7VpyJt3FdCVs8\/Q9nurTaO+OjFmym3Jzqdtew+a\/\/k4LrfjQHOiE9vjuBxvtq47qo+YOrku3sedjevj8\/B29a3FP43Nx5n3S\/XZ89hz8f9cvxs3d9atu0tMzBwTw8HmwnVem3gp8TnXx3nx+Tl\/fm9zNXDaXWJysZrsPHjEHGPjsZ\/bPjP\/0lFTlwsx3mc1sfUn0xt\/CSh88mwoAOTl5VFDQwOVl5dTd3c3jY6OJqYBnGjtD56iF5\/+Ef3znaUr8e4S03i5Q9gvpPCVakt2wbI\/U8XHuH+7Ptkx3DHcTuq11pa5MTe97Oqc7Nx2dML\/u6MVfv\/AAw\/Qm3\/\/F126exFK7hdpvMfaq66NhWP01u\/dl+wc3Lnd\/LkMx+1evZNp4q2f3\/O5GFLfuG8fXbhwgWZmov1FIPYjynnk5+dTLBaj2tpaX73YlJOT86mfvOMRwEoA4EQ7OjrMVAAbFIACnykQj8eppaXFV0kCAcBKUwDOjiHA\/7BBASjwmQI8CvN7JOY7AFZbBIThUAAKhKeA7wDg1OxtwIWFBTOvsQuA4aWNmqEAFGAFAgEApIYCUECmAqECYKUHhKTJxVMZXpDJysoyoV28eNHc2XBHOPzafdDJ3vXYuXMnTU9PL3v+Iej8Wltbb3kGI5X+fOyhQ4dMiGfOnKH29nbzWkI+3jz27NlDx44do4KCAhPj\/Py8WVQeHx8niXm4MXlHxGH4ERoAorY2kKwDcYNbKY8TJ07Q7OwsHT9+3DTS06dPL3sKMigI2IblQihV3O6zGhyffYiLO1TY+STLg6HkfbbEwsrul5KHN1bOh+9+MbCKioqSPjDntx+hAWC1B4SC6hxrrce9nemWSZXH1NTUsmceUpVfa\/3pHsfgKiwsNMXt3Rh+CjNV3HzLtqamxjTKDz\/8MAGuc+fOhZpPqjxckDGk7MbHS8zD9dGNne\/3J7tb5rcfoQJgpecD0m3wfpTzDjPdoVuq5xy4w7hXJrfD2ceg\/Yg11Tm99aeKm8vbx7X5NY9c3nvvPTp58qSIfJLlYb9n4k5Z3BGbxDzs1NH2gbD8AADS6IXu1YXJnQxkAEAawq6hyEogdZ80lQ4y75RSJQBWe0BoDe0hlEN46Nbc3Ez9\/f1UUlKSdOgmZQpgBUp25QxjyHm7hq0EADtS4xHLBx98IHYKkGw6GNaULLQRQJQWAblh9fT00KlTp8wzDK5ZpaWlKb\/tGPaiWaq1Cp6CRHER0A6bvWsZvJ\/vVLhgZgBLWwS08dt4U60HuN+a3bCLgFYMnr9F4QEh9zZgqts33v3ubTP3tuHtXgXTKZ\/sypnqAS33VlWq25ph5ePNw7s+4962lJaH91Yy++jetgzDj9BGAOk0YpSBAlAgswoAAJnVE2eDApFSAACIlF0IFgpkVgEAILN64mxQIFIKAACRsgvBQoHMKvB\/fYUGizYfQCMAAAAASUVORK5CYII=","height":154,"width":256}}
%---
%[output:24248189]
%   data: {"dataType":"text","outputData":{"text":"\ntf1 =\n \n  From input \"u1\" to output \"y1\":\n    53.95\n  ---------\n  s + 54.49\n \nName: M6_tf\nContinuous-time identified transfer function.\n\nParameterization:\n   Number of poles: 1   Number of zeros: 0\n   Number of free coefficients: 2\n   Use \"tfdata\", \"getpvec\", \"getcov\" for parameters and their uncertainties.\n\nStatus:                                              \nEstimated using TFEST on time domain data \"M6_step2\".\nFit to estimation data: 94.41% (stability enforced)  \nFPE: 0.2945, MSE: 0.2936                             \n \n<a href=\"matlab:if exist('tf1','var'), if isa(tf1,'idtf'), disp(' '); disp(get(tf1)), else disp(' '); disp('Unable to display properties for variable tf1 because it is no longer a\/an idtf object.');, end, else matlab.graphics.internal.getForDisplay('tf1'), end\">Model Properties<\/a>\n","truncated":false}}
%---
%[output:4fcef7bc]
%   data: {"dataType":"textualVariable","outputData":{"name":"JM6","value":"0.0185"}}
%---
%[output:4ebc8202]
%   data: {"dataType":"textualVariable","outputData":{"name":"bM6","value":"0.0101"}}
%---
%[output:89866e1f]
%   data: {"dataType":"text","outputData":{"text":"\ntf1 =\n \n  From input \"u1\" to output \"y1\":\n    55.66\n  ---------\n  s + 56.21\n \nName: M5_tf\nContinuous-time identified transfer function.\n\nParameterization:\n   Number of poles: 1   Number of zeros: 0\n   Number of free coefficients: 2\n   Use \"tfdata\", \"getpvec\", \"getcov\" for parameters and their uncertainties.\n\nStatus:                                              \nEstimated using TFEST on time domain data \"M5_step2\".\nFit to estimation data: 94.16% (stability enforced)  \nFPE: 0.3231, MSE: 0.3221                             \n \n<a href=\"matlab:if exist('tf1','var'), if isa(tf1,'idtf'), disp(' '); disp(get(tf1)), else disp(' '); disp('Unable to display properties for variable tf1 because it is no longer a\/an idtf object.');, end, else matlab.graphics.internal.getForDisplay('tf1'), end\">Model Properties<\/a>\n","truncated":false}}
%---
%[output:7f9e0329]
%   data: {"dataType":"textualVariable","outputData":{"name":"JM5","value":"0.0180"}}
%---
%[output:6256d6c2]
%   data: {"dataType":"textualVariable","outputData":{"name":"bM5","value":"0.0098"}}
%---
%[output:8c0fffb9]
%   data: {"dataType":"text","outputData":{"text":"\ntf1 =\n \n  From input \"u1\" to output \"y1\":\n    57.19\n  ---------\n  s + 57.74\n \nName: M2_tf\nContinuous-time identified transfer function.\n\nParameterization:\n   Number of poles: 1   Number of zeros: 0\n   Number of free coefficients: 2\n   Use \"tfdata\", \"getpvec\", \"getcov\" for parameters and their uncertainties.\n\nStatus:                                              \nEstimated using TFEST on time domain data \"M3_step2\".\nFit to estimation data: 93.33% (stability enforced)  \nFPE: 0.4217, MSE: 0.4204                             \n \n<a href=\"matlab:if exist('tf1','var'), if isa(tf1,'idtf'), disp(' '); disp(get(tf1)), else disp(' '); disp('Unable to display properties for variable tf1 because it is no longer a\/an idtf object.');, end, else matlab.graphics.internal.getForDisplay('tf1'), end\">Model Properties<\/a>\n","truncated":false}}
%---
%[output:405fb475]
%   data: {"dataType":"textualVariable","outputData":{"name":"JM2","value":"0.0175"}}
%---
%[output:9dc63e61]
%   data: {"dataType":"textualVariable","outputData":{"name":"bM2","value":"0.0097"}}
%---
