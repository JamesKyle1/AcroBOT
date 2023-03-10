function [t,x,te,xe,ie] = simAcroBOT(x0,tspan,SYS)

% Initialize function specific variables
contactMode = [1];
n = 50; % number of points in trajectory


% Extract variables passed into function
tstart = tspan(1);
tfinal = tspan(end);
dt = tspan(2) - tspan(1);

currSYS = SYS;
alpha = currSYS.alpha;
archRegion = currSYS.archRegion;
hollowRegion = currSYS.hollowRegion;


%Initialize out variables
t = tstart;
x = x0';
te = [];
xe = [];
ie = [];

while tstart < tfinal
    % Tell ode45 what event function to use and set max step size to make sure
    % we don't miss a zero crossing
    options = odeset('Events', @(t,x) guardFunctions(t,x,contactMode,hollowRegion,archRegion),'MaxStep',0.01);
%     options = odeset('MaxStep',0.01);

    % Initialize simulation time vector
    tspan = [tstart:dt:tfinal];
    
    % Simulate with ode45
    [tout, xout, teout, xeout, ieout] = ode45(@(t,x) dynamics(t,x,contactMode,currSYS),tspan,x0,options);
    
    % Sometimes the events function will record a nonterminal event if the
    % initial condition is a zero. We want to ignore this, so we will only
    % use the last row in the terminal state, time, and index.
    if ~isempty(ieout)
        teout = teout(end,:);
        xeout = xeout(end,:);
        ieout = ieout(end,:);
    end
    
    % Log output
    nt = length(tout);
    t = [t; tout(2:nt)];
    x = [x; xout(2:nt,:)];
    te = [te; teout];
    xe = [xe; xeout];
    ie = [ie; ieout];
%     transitionPts = [transitionPts; length(x)];

     % Break if done with simulation
    if isempty(ieout)
        disp('Final time reached');
        break; % abort if simulation has completed
    end


    %Update contact mode
    q = xeout(1:3)';
    
    if ieout == 2
        contactMode = 2;
        [trajTime, trajPos] = minJerkTraj(q(2:3), alpha.*ones(2,1), archRegion(1), archRegion(2), n);
%         trajPosTracker = [trajPosTracker; trajPos'];
%         trajTimeTracker = [trajTimeTracker; trajTime'];
        currSYS.trajectoryPos = trajPos;
        currSYS.trajectoryTime = trajTime;
    elseif ieout == 4
        contactMode = 4;
        [trajTime, trajPos] = minJerkTraj(q(2:3), -alpha.*ones(2,1), hollowRegion(1), hollowRegion(2), n);
%         trajPosTracker = [trajPosTracker; trajPos'];
%         trajTimeTracker = [trajTimeTracker; trajTime'];
        currSYS.trajectoryPos = trajPos;
        currSYS.trajectoryTime = trajTime;
    else
        contactMode = 1;
        [currSYS.trajectoryTime, currSYS.trajectoryPos] = minJerkTraj(q(2:3), zeros(2,1), hollowRegion(1), hollowRegion(2), n);
    end
    

    % Update initial conditions for next iteration
    x0 = xeout;
    tstart = t(end);

end




end