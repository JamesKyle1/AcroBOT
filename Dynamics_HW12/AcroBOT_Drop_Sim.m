clear all;clc;close all;
%% Initialization

% Define start and stop times, set a dt to keep outputs at constant time
% interval
tstart = 0;
tfinal = 50;
dt = 0.01;

% Initialize state and contact mode
q0 = [pi/2;0;0];
disp(['Initial condition: [', num2str(q0'), ']''.'])
dq0 = [-1;0;0];
x0 = [q0;dq0];
contactMode = [];
gymnastSYS.m = [1;1;1];
gymnastSYS.l = [1;1;1];
gymnastSYS.c = [0.9;0.9;0.9];
gymnastSYS.dt = dt;
%% Main Loop

% Tell ode45 what event function to use and set max step size to make sure
% we don't miss a zero crossing
options = odeset('Events', @guardFunctions,'MaxStep',0.1);

% Initialize simulation time vector
tspan = [tstart:dt:tfinal];
    
% Simulate
[t,x] = ode45(@(t,y) dynamics(t,y,gymnastSYS),tspan,x0,options);

%% 

th1 = x(:,1);
th2 = x(:,2);
th3 = x(:,3);
figure();
hold on
% plot(t,th1,'DisplayName','$\theta_1$');
plot(t,th2,'DisplayName','$\theta_2$');
plot(t,th3,'DisplayName','$\theta_3$');
hold off
ylabel('$\theta [rad]$');
xlabel('Time [s]');
legend('Interpreter','Latex');


figure();
hold on
% plot(t,th1,'DisplayName','$\theta_1$');
plot(t,x(:,4),'DisplayName','$\dot\theta_2$');
% plot(t,th3,'DisplayName','$\theta_3$');
hold off
ylabel('$\theta [rad]$');
xlabel('Time [s]');
legend('Interpreter','Latex');

%% 

% animateAcroBOT(x, dt, gymnastSYS);
