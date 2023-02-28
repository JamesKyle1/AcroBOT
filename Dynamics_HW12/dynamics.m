function dx = dynamics(t,x,SYS)
% Initialize time derivative of state vector as column vector
dx = zeros(length(x),1);

% Split up state vector into generalized coordinates and velocities
q = x(1:3);
dq = x(4:6);

% System Parameters
params = [SYS.m; SYS.l; SYS.c];

% Solve the equations of motion
ddq = solveEOM(t,x,params);

% Time derivative of generalized positions are the generalized velocities
dx(1:3) = dq;

% Time derivative of generalized velocities are accelerations (from EOM)
dx(4:6) = ddq;

end