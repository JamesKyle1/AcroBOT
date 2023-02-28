clear;clc;close all;

% Define start and stop times, set a dt to keep outputs at constant time
% interval
tstart = 0;
tfinal = 3;
dt = 0.01;

% Initialize state and contact mode
% q0 = [0.2;1];
q0 = [1;5];
dq0 = [0;0];
x0 = [q0;dq0];
contactMode = [];

%% Main Loop



% Initialize output arrays
tout = [];
xout = [];
teout = [];
xeout = [];
ieout = [];

% Main simulation loop
while tstart < tfinal
    % Tell ode45 what event function to use and set max step size to make sure
    % we don't miss a zero crossing
    options = odeset('Events', @(t,x) guardFunctions(t,x,contactMode),'MaxStep',0.01);

    % Initialize simulation time vector
    tspan = [tstart:dt:tfinal];
    
    % Simulate with ode45
    [t, x, te, xe, ie] = ode45(@(t,x) dynamics(t,x,contactMode),tspan,x0,options);

    
    % Sometimes the events function will record a nonterminal event if the
    % initial condition is a zero. We want to ignore this, so we will only
    % use the last row in the terminal state, time, and index.
    if ~isempty(ie)
        te = te(end,:);
        xe = xe(end,:);
        ie = ie(end,:);
    end
    
    % Log output
    nt = length(t);
    tout = [tout; t(2:nt)];
    xout = [xout; x(2:nt,:)];
    teout = [teout; te];
    xeout = [xeout; xe];
    ieout = [ieout; ie];
    
    % Quit if simulation completes
    if isempty(ie) 
        disp('Final time reached');
        break; % abort if simulation has completed
    end
    
    % If flag was caused by a_i < 0 (i not in contact mode), compute the
    % proper contact mode via IV complemetarity
    contactModeIV = [];
    if sum(ismember(1:3,ie))
        contactModeIV = compIV(xe);
    end
    
    
    % Check to see if there should be liftoff (positive lambda), if so
    % compute the proper contact mode via FA complementarity
    contactModeFA = [];
    if ~sum(ismember(1:3,ie))
        contactModeFA = compFA(xe);
    end

    contactMode = unique([contactModeIV contactModeFA]);
    
    %Parameters
    m = 1;
    n = length(xe)/2;
    c = length(contactMode);

    x = xe(1);
    y = xe(2);
    dx = xe(3);
    dy = xe(4);
    q = [x;y];
    dq = [dx;dy];
    
    
    %Constraints
    A = [   0        1    ;
            1        1    ;
         2*x - 4  2*y - 2 ];
    A = A(contactMode,:);
    dA = [   0   0 ;
             0   0 ;
             2   2 ];
    dA = dA(contactMode,:);

    %Setting up dynamics
    M = [m  0 ;
         0  m ];
    
    
    block = [M         A'    ;
             A   zeros(c, c) ];
    blockInv = inv(block);
    M_dagger = blockInv(1:n,1:n);
    A_dagger = blockInv(n+1:n+c,1:n);
    Lambda = blockInv(n+1:n+c,n+1:n+c);

    dq_p = dq - A_dagger'*A*dq;

    % Update initial conditions for next iteration
    x0 = [xe(1:2)'; dq_p];
    tstart = t(end);
    
    % Stop if the particle comes to a rest
    if all(abs(dq_p)<1e-6)
        break;
    end
end

% This function shows animation for the simulation, don't modify it
animateHW11(xout, dt);

