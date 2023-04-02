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


% figure();
% th1d = x(:,4);
% set(gcf,'WindowState','maximized');
% hold on
% plot(t,th1d,'linewidth',2,'DisplayName','$\dot{\theta_1}$');
% hold off
% ylabel('$\dot{\theta_1} [rad]$');
% xlabel('Time [s]');
% legend('Interpreter','Latex');


%% Animating Test Simulation

animateAcroBOT(x, dt, gymnastSYS);


%% =====================================================================
%  ==============  Building Parameter Change Vectors  ==================
%  =====================================================================

% Lengths iterations
dl = 0.03;

L = 27/12; % [ft]
L = L*0.3048; % converting to [m]

l_min = 0.1;
l_max = L-2*l_min;
l1_vector = l_min:dl:l_max;

n = length(l1_vector);


lengths = zeros(1,3);
for i = 1:n
    % iterating through l1 lengths
    l1 = l_min + dl*(i-1);

    for j = 1:length(l_min:dl:L-l1-l_min)
        % iterating through l2/l3 ratios
        l2 = l_min + dl*(j-1);
        l3 = L-l1-l2;
        lengths = [lengths; l1 l2 l3];
    end
end

lengths = lengths(2:end,:);


% COM placement iterations
dr = 0.1;
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

tic
[t,x,te,xe,ie] = simAcroBOT(x0,tspan,gymnastSYS);
dt = toc;

Min = [l_min; r_min; m_min];
Max = [l_max; r_max; m_max];
Step = [dl; dr; dm];
Variable = ["Link Length";"COM Placement";"Mass"];

disp(table(Variable, Min, Max, Step))
fprintf(" Number of Trials: " + n_iter)
fprintf("\n Sim Iter Time Est.: " + dt + " s")
fprintf("\n Full Run Time Est.: " + mod(n_iter*dt/60,60) + " min")
fprintf("\n");


%% ================  Building Simulaiton Loops  ========================

tstart = 0;
tfinal = 7;
dt = 0.01;


% Initialize state and contact mode
q0 = [0;0;0];
dq0 = [0;0;0];
x0 = [q0;dq0];

gymnastSYS.dt = dt;
gymnastSYS.m = [0.1564;0.23764;0.1564]; %Setting mass [kg]


% Initialize simulation time vector
tspan = tstart:dt:tfinal;


% Initialize control input regions
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
%% 

% Initializing folder for data
folderPath = "Apr_2_2023_Single_COM_Set_2/";

if exist(folderPath,"dir") ~= 0
    error("File name already exists");
else
    evalin("base",'mkdir ' + folderPath);
end


% Running simulation over range of parameters
% n_lengths = length(lengths);
n_lengths = length(lengths);
% n_COM = length(COM_vec);
n_COM = 1;

disp("Simulating Parameter Changes...");
tic
for i = 1:n_lengths
    li = lengths(i,:)';
%     li = [0.2200    0.2200    0.2458]';

    for j = 1:n_COM
        disp("Trial #: " + ((i-1)*n_COM+j) + "/" + n_lengths*n_COM);

%         ci = COM_vec(j,:)'.*li;
        ci = [0.5; 0.5; 0.5].*li;

        % Create new file name
        fileName = "L_" + i + "_COM_" + j;
        filePath = folderPath + fileName;

        params = [li; ci; gymnastSYS.m]';

        % Simulate and write data to file
        gymnastSYS.l = li;
        gymnastSYS.c = ci;

        [t,x,te,xe,ie] = simAcroBOT(x0,tspan,gymnastSYS);

        writeAcroBOTData([t x],params,filePath);
        
    end


end
toc

disp("Simulations Complete");




