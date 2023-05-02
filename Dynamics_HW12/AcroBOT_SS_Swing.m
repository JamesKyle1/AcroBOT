clear all;clc;close all;
%% Initialization

% Define start and stop times, set a dt to keep outputs at constant time
% interval
tstart = 0;
tfinal = 17;
dt = 0.01;


% Initialize state and contact mode
q0 = [0;0;0];
disp(['Initial condition: [', num2str(q0'), ']''.'])
dq0 = [0;0;0];
x0 = [q0;dq0];
gymnastSYS.m = [0.1564;0.23764;0.1564];
gymnastSYS.l = [0.254;0.254;0.254];
% gymnastSYS.l = lengths(1,:)';
gymnastSYS.c = gymnastSYS.l.*0.1;
% gymnastSYS.c = COM_vec(28,:)';
gymnastSYS.dt = dt;
%% Main Loop

% Initialize simulation time vector
tspan = tstart:dt:tfinal;

t = tstart;
x = x0';
te = [];
xe = [];
ie = [];
transitionPts = [];
trajPosTracker = zeros(1,2);
trajTimeTracker = zeros(1,2);


alpha = pi/3;
beta = pi/6;
gamma = pi/4;

n = 50; % number of points in trajectory

archRegion = 2*pi - [beta;beta+gamma];
hollowRegion = pi + [beta+gamma;beta];

gymnastSYS.alpha = alpha;
gymnastSYS.beta = beta;
gymnastSYS.gamma = gamma;
gymnastSYS.archRegion = archRegion;
gymnastSYS.hollowRegion = hollowRegion;


% Simulate
tic
[t,x,te,xe,ie] = simAcroBOT(x0,tspan,gymnastSYS);
toc


%%

th1 = x(:,1);
th2 = x(:,2);
th3 = x(:,3);


figure();
set(gcf,'WindowState','maximized');
hold on
plot(t,th2,'linewidth',2,'DisplayName','$\theta_2$');
plot(t,th3,'linewidth',2,'DisplayName','$\theta_3$');
plot([t(1) t(end)],alpha.*ones(2,1),'k--','linewidth',2,'HandleVisibility','off');
plot([t(1) t(end)],-alpha.*ones(2,1),'k--','linewidth',2,'HandleVisibility','off');
hold off
ylabel('$\theta [rad]$');
xlabel('Time [s]');
legend('Interpreter','Latex');


%%

animateAcroBOT(x, dt, gymnastSYS);




