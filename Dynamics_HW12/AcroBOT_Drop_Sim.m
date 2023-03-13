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


%% Plotting Test Simulation

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


%% Animating Test Simulation

% animateAcroBOT(x, dt, gymnastSYS);


%% Simulating Parameter Changes


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
    plot(currTime,currXout(:,2),'DisplayName',dispName);
    
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
    plot(currTime,currXout(:,3),'DisplayName',dispName);
    
end
hold off;
legend()
title('$\theta_3$','Interpreter','Latex')
ylabel('$\theta [rad]$');
xlabel('Time [s]');

%% Aggregating Parameter Change Vectors

% Lengths iterations
dl = 0.05;

L = 2.3; % [ft]
L = L*0.3048; % converting to [m]

l_min = 0.1;
l_max = 0.5;
l1_vector = l_min:dl:l_max;

n = length(l1_vector);


lengths = zeros(1,3);
for i = 1:n
    % iterating through l1 lengths
    l1 = l_min + dl*(i-1);

    for j = 1:(n-i)
        % iterating through l2/l3 ratios
        l2 = l_min + dl*(j-1);
        l3 = L-l1-l2;
        lengths = [lengths; l1 l2 l3];
    end
end

lengths = lengths(2:end,:);


% COM placement iterations
dr = 0.2;
r_min = 0.1;
r_max = 0.9;
ratios = r_min:dr:r_max;

p = length(ratios);

COM_vec = zeros(1,3);
for i = 1:p
    % iterating through l1 lengths
    c1 = 1*ratios(i);

    for j = 1:p
        % iterating through l2/l3 ratios
        c2 = 1*ratios(j);

        for k = 1:p
            % iterating through l2/l3 ratios
            c3 = 1*ratios(k);

            COM_vec = [COM_vec; c1 c2 c3];
        end
    end
end

COM_vec = COM_vec(2:end,:);


% Mass iterations
dm = 0.2;
m_min = 0.2;
m_max = 0.6;
masses = m_min:dm:m_max;

h = length(masses);

mass_vec = zeros(1,3);
for i = 1:h
    % iterating through l1 lengths
    c1 = 1*ratios(i);

    for j = 1:h
        % iterating through l2/l3 ratios
        c2 = 1*ratios(j);

        for k = 1:h
            % iterating through l2/l3 ratios
            c3 = 1*ratios(k);

            mass_vec = [mass_vec; c1 c2 c3];
        end
    end
end

mass_vec = mass_vec(2:end,:);

n_iter = length(COM_vec)*length(lengths);

% tic
% [t,x,te,xe,ie] = simAcroBOT(x0,tspan,gymnastSYS);
% dt = toc;
% 
% Min = [l_min; r_min; m_min];
% Max = [l_max; r_max; m_max];
% Step = [dl; dr; dm];
% Variable = ["Link Length";"COM Placement";"Mass"];
% 
% disp(table(Variable, Min, Max, Step))
% fprintf(" Number of Trials: " + n_iter)
% fprintf("\n Sim Iter Time Est.: " + dt + " s")
% fprintf("\n Full Run Time Est.: " + mod(n_iter*dt/60,60) + " min")
% fprintf("\n");

%% ======================================================================
% Experimenting with saving data from experiment

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


% Initialize simulation time vector
tspan = tstart:dt:tfinal;

alpha = pi/4;
beta = pi/6;
gamma = pi/4;

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

folderPath = "expTest/";
fileName = "test2";
filePath = folderPath + fileName;
params = [gymnastSYS.l' gymnastSYS.c' gymnastSYS.m'];

writeAcroBOTData([t x],params,filePath);
toc
