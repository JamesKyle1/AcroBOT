clear all;clc;close all;
%% Initialization

% Define start and stop times, set a dt to keep outputs at constant time
% interval
tstart = 0;
tfinal = 10;
dt = 0.01;

% Initialize state and contact mode
q0 = [pi/4;0;0];
disp(['Initial condition: [', num2str(q0'), ']''.'])
dq0 = [0;0;0];
x0 = [q0;dq0];
contactMode = [1];
gymnastSYS.m = [1;1;1];
gymnastSYS.l = [1;1;1];
gymnastSYS.c = [0.1;0.1;0.1];
gymnastSYS.dt = dt;
%% Main Loop

% Initialize simulation time vector
tspan = tstart:dt:tfinal;

t = tstart;
x = x0';
te = [];
xe = [];
ie = [];


alpha = pi/3;
beta = pi/6;
gamma = pi/4;

archRegion = 2*pi - [beta;beta+gamma];
hollowRegion = pi + [beta+gamma;beta];
    
% Simulate
while tstart < tfinal
    % Tell ode45 what event function to use and set max step size to make sure
    % we don't miss a zero crossing
    options = odeset('Events', @(t,x) guardFunctions(t,x,contactMode,hollowRegion,archRegion),'MaxStep',0.01);

    % Initialize simulation time vector
    tspan = [tstart:dt:tfinal];
    
    % Simulate with ode45
    [tout, xout, teout, xeout, ieout] = ode45(@(t,x) dynamics(t,x,contactMode,gymnastSYS),tspan,x0,options);

    
    % Sometimes the events function will record a nonterminal event if the
    % initial condition is a zero. We want to ignore this, so we will only
    % use the last row in the terminal state, time, and index.
    if ~isempty(ieout)
        teout = teout(end,:);
        xeout = xeout(end,:);
        ieout = ieout(end,:);
    else
        disp('Final time reached');
        break; % abort if simulation has completed
    end
    
    % Log output
    nt = length(tout);
    t = [t; tout(2:nt)];
    x = [x; xout(2:nt,:)];
    te = [te; teout];
    xe = [xe; xeout];
    ie = [ie; ieout];


    %Update contact mode
    q = xeout(1:3)';
    n = 20;

    
    if ieout == 2
        contactMode = 2;
        [gymnastSYS.trajectoryTime, gymnastSYS.trajectoryPos] = minJerkTraj(q(2:3), alpha.*ones(2,1), archRegion(1), archRegion(2), n);
    elseif ieout == 4
        contactMode = 3;
        [gymnastSYS.trajectoryTime, gymnastSYS.trajectoryPos] = minJerkTraj(q(2:3), -alpha.*ones(2,1), hollowRegion(1), hollowRegion(2), n);
    else
        contactMode = 1;
        [gymnastSYS.trajectoryTime, gymnastSYS.trajectoryPos] = minJerkTraj(q(2:3), zeros(2,1), hollowRegion(1), hollowRegion(2), n);
    end
    

    % Update initial conditions for next iteration
    x0 = xeout;
    tstart = t(end);

end


%% 

th1 = x(:,1);
th2 = x(:,2);
th3 = x(:,3);
figure();
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
