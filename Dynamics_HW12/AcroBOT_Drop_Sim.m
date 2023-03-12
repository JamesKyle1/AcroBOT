clear all;clc;close all;
%% Initialization

% Define start and stop times, set a dt to keep outputs at constant time
% interval
tstart = 0;
tfinal = 10;
dt = 0.01;


% Initialize state and contact mode
q0 = [0;0;0];
disp(['Initial condition: [', num2str(q0'), ']''.'])
dq0 = [0;0;0];
x0 = [q0;dq0];
gymnastSYS.m = [0.3;0.3;0.3];
gymnastSYS.l = [0.254;0.254;0.254];
gymnastSYS.c = gymnastSYS.l.*0.1;
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


alpha = pi/4;
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
% subplot(1,2,1);
hold on
plot(t,th2,'linewidth',2,'DisplayName','$\theta_2$');
plot(t,th3,'linewidth',2,'DisplayName','$\theta_3$');
plot([t(1) t(end)],alpha.*ones(2,1),'k--','linewidth',2,'HandleVisibility','off');
plot([t(1) t(end)],-alpha.*ones(2,1),'k--','linewidth',2,'HandleVisibility','off');
hold off
ylabel('$\theta [rad]$');
xlabel('Time [s]');
legend('Interpreter','Latex');

% subplot(1,2,2);
% hold on
% plot(t,tau(:,2),'linewidth',2,'DisplayName','$\theta_2$');
% plot(t,tau(:,3),'linewidth',2,'DisplayName','$\theta_3$');
% hold off
% ylabel('$Input[Nm]$');
% xlabel('Time [s]');
% legend('Interpreter','Latex');


% span = transitionPts(1):transitionPts(2);
% tOverTraj = t(span);
% 
% figure();
% hold on
% plot(tOverTraj,th2(span),'linewidth',2,'DisplayName','$\theta_2$');
% plot(linspace(tOverTraj(1),tOverTraj(end),length(trajPosTracker(1,:))),trajPosTracker(1,:)','linewidth',2,'DisplayName','Trajectory');
% hold off
% ylabel('$\theta [rad]$');
% xlabel('Time [s]');
% legend('Interpreter','Latex');

%% 

% animateAcroBOT(x, dt, gymnastSYS);


%% Simulating parameter changes


tic
c_vector = 0.1:0.1:0.9;
trials = cell(length(c_vector),1);
for i = 1:length(c_vector)
    gymnastSYS.c = c_vector(i).*gymnastSYS.l;
    [t,x,te,xe,ie] = simAcroBOT(x0,tspan,gymnastSYS);
    trials{i} = {t,x,te,xe,ie};
end
toc

%% 

foo = trials{1}(1);
N = length(foo{1});

t = zeros(N,length(trials));
th1 = zeros(N,length(trials));
th2 = zeros(N,length(trials));
th3 = zeros(N,length(trials));

figure();
subplot(2,1,1);
hold on;
for i = 1:length(trials)
    currTime = trials{i}{1};
    currXout = trials{i}{2};
    
    dispName = string("C = " + c_vector(i));
    plot(currTime,currXout(:,5),'DisplayName',dispName);
    
end
hold off;
legend()
title('$\theta_2$','Interpreter','Latex')
ylabel('$\theta [rad]$');
xlabel('Time [s]');

subplot(2,1,2);
hold on;
for i = 1:length(trials)
    currTime = trials{i}{1};
    currXout = trials{i}{2};
    
    dispName = string("C = " + c_vector(i));
    plot(currTime,currXout(:,6),'DisplayName',dispName);
    
end
hold off;
legend()
title('$\theta_3$','Interpreter','Latex')
ylabel('$\theta [rad]$');
xlabel('Time [s]');
