clc
clear all

waypoints = [ 0 0 0 0;
    0 0 -10 1;
    10 10 -10 1;
    10 30 -20 1;
    30 40 -30 1];
timepoints = [0 5 10 15 20];

SimulationTime = 20;
timestep = 0.01;

trajectory_coef = min_snap_Coef(waypoints,timepoints);

X =[];
Y = [];
Z = [];
Psi = [];
dX = [];
dY = [];
dZ = [];
dPsi = [];
ddX = [];
ddY = [];
ddZ = [];
ddPsi = [];
dddX = [];
dddY = [];
dddZ = [];
ddddX = [];
ddddY = [];
ddddZ = [];

tX = 0;
tY = 0;
tZ = 0;
tPsi = 0;
tdX = 0;
tdY = 0;
tdZ = 0;
tdPsi = 0;
tddX = 0;
tddY = 0;
tddZ = 0;
tddPsi = 0;

time = 0:0.01:SimulationTime;
for t = time
[tX,tY,tZ,tPsi,tdX,tdY,tdZ,tdPsi,tddX,tddY,tddZ,tddPsi,tdddX,tdddY,tdddZ,tddddX,tddddY,tddddZ] = getTrajectory(t,trajectory_coef,timepoints);
X = [X tX];
Y = [Y tY];
Z = [Z tZ];
Psi = [Psi tPsi];
dX = [dX tdX];
dY = [dY tdY];
dZ = [dZ tdZ];
dPsi = [dPsi tdPsi];
ddX = [ddX tddX];
ddY = [ddY tddY];
ddZ = [ddZ tddZ];
ddPsi = [ddPsi tddPsi];
dddX = [dddX tdddX];
dddY = [dddY tdddY];
dddZ = [dddZ tdddZ];
ddddX = [ddddX tddddX];
ddddY = [ddddY tddddY];
ddddZ = [ddddZ tddddZ];

end

plot(time, X)
xlabel('Time (s)')
ylabel('X position (m)')
title('Trajectory of X axis')
grid on