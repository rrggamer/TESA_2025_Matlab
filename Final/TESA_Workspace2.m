clc
clear all


load data.mat


%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%%%% Create Variable in Simulick Workspace %%%%
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%



% properties of drones
m = 4.34; 
g = 9.81;
dIdt = zeros(3,3);
I = eye(3,3);
I(1,1) = 0.0820;
I(2,2) = 0.0845;
I(3,3) = 0.1377;
L = 0.315; %length of arms

% motor coefficient
cf = 8.004e-4;

% K matrix to convert motor speed to force and torques u = K . omega2
K = [1 1 1 1;
    0 -L 0 L;
    L 0 -L 0;
    -cf cf -cf cf];

Kp = eye(3,3);
Kp = 1*Kp;
Kv = eye(3,3);
Kv = 1*Kv;
Kr = eye(3,3);
Kr = 1*Kr;
Kw = eye(3,3);
Kw = 1*Kw;



time = 0:timestep:SimulationTime;
inputStructure.time = time;
inputStructure.signals(1).values = X';
inputStructure.signals(1).dimensions = 1;
inputStructure.signals(2).values = Y';
inputStructure.signals(2).dimensions = 1;
inputStructure.signals(3).values = Z';
inputStructure.signals(3).dimensions = 1;

inputStructure.signals(4).values = dX';
inputStructure.signals(4).dimensions = 1;
inputStructure.signals(5).values = dY';
inputStructure.signals(5).dimensions = 1;
inputStructure.signals(6).values = dZ';
inputStructure.signals(6).dimensions = 1;

inputStructure.signals(7).values = ddX';
inputStructure.signals(7).dimensions = 1;
inputStructure.signals(8).values = ddY';
inputStructure.signals(8).dimensions = 1;
inputStructure.signals(9).values = ddZ';
inputStructure.signals(9).dimensions = 1;

inputStructure.signals(10).values = dddX';
inputStructure.signals(10).dimensions = 1;
inputStructure.signals(11).values = dddY';
inputStructure.signals(11).dimensions = 1;
inputStructure.signals(12).values = dddZ';
inputStructure.signals(12).dimensions = 1;

inputStructure.signals(13).values = ddddX';
inputStructure.signals(13).dimensions = 1;
inputStructure.signals(14).values = ddddY';
inputStructure.signals(14).dimensions = 1;
inputStructure.signals(15).values = ddddZ';
inputStructure.signals(15).dimensions = 1;


inputStructure.signals(16).values = Psi';
inputStructure.signals(16).dimensions = 1;
inputStructure.signals(17).values = dPsi';
inputStructure.signals(17).dimensions = 1;
inputStructure.signals(18).values = ddPsi';
inputStructure.signals(18).dimensions = 1;



%out = sim('quadrotorsmodel2.slx');
out = sim('Final.slx','SimulationMode','normal');

Xe = out.yout{1}.Values.Data(:,1)';
Ye = out.yout{1}.Values.Data(:,2)';
Ze = out.yout{1}.Values.Data(:,3)';

size(Xe);

figure;
subplot(3,1,1)
plot(time, X, 'b', 'LineWidth', 1.5); hold on;
plot(time, Xe, 'r--', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('X [m]');
legend('Desired', 'Actual'); grid on; title('X-axis');

subplot(3,1,2)
plot(time, Y, 'b', 'LineWidth', 1.5); hold on;
plot(time, Ye, 'r--', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Y [m]');
legend('Desired', 'Actual'); grid on; title('Y-axis');

subplot(3,1,3)
plot(time, Z, 'b', 'LineWidth', 1.5); hold on;
plot(time, Ze, 'r--', 'LineWidth', 1.2);
xlabel('Time [s]'); ylabel('Z [m]');
legend('Desired', 'Actual'); grid on; title('Z-axis');

figure;
plot(time, Xe - X, 'r', 'LineWidth', 1.2); hold on;
plot(time, Ye - Y, 'g', 'LineWidth', 1.2);
plot(time, Ze - Z, 'b', 'LineWidth', 1.2);
xlabel('Time [s]');
ylabel('Error [m]');
legend('X error', 'Y error', 'Z error');
title('Position Tracking Error');
grid on;


f2 = figure;
plot3(X,Y,Z,Xe,Ye,Ze);
f2.CurrentAxes.ZDir = 'Reverse';
xlabel('x')
ylabel('y')
zlabel('-z')


