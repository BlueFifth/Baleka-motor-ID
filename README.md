# Baleka-motor-ID
System identification for Baleka's QDD BLDC Motors (AK10-9 V3.0)

Running the steps in this repo assumes you've previously completed 
actuation tuning as per the [Baleka actuation & proprioception repo](https://github.com/BlueFifth/Baleka-actuation-proprioception)

Make sure to add the torque scaling factors to the step test files (StepParams.m)

First run the step tests. For the given experiment the matrix of step params
in StepParams.m was run through twice. 

Following this the data was used for LSQ optimization using the model and real data.


Start by using high osc samples with low step time - i.e. don't weigh
the steady state values highly. This will ideally find better local minima

Change bounds!
Ct<= Bt must be inforced by limits not constraints [read more here](https://www.mathworks.com/help/optim/ug/iterations-can-violate-constraints.html)

Results: (in unscaled units (proper measurements))
M2 simple:
J = 0.01544
b = 0.01181

M2 full
J = 0.015369
b = 0.0414
Bt = 0.3453
Ct = 0.1394

M3 simple:
J = 0.015918
b = 0.01399

M3 full
J = 0.015918
b = 0.000018
Bt = 0.8037
Ct = 0.2798

M5 simple:
J = 0.015617
b = 0.01350

M5 full
J = 0.011016
b = 0.0000019
Bt = 0.3028
Ct = 0.2288

M6 simple:
J = 0.016061
b = 0.01292

M6 full
J = 0.016038
b = 0.0000014
Bt = 0.4402
Ct = 0.2418