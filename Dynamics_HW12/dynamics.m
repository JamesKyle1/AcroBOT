function dx = dynamics(t,x,contactMode,SYS)
% Initialize time derivative of state vector as column vector
dx = zeros(length(x),1);

% Split up state vector into generalized coordinates and velocities
q = x(1:3);
dq = x(4:6);


% System Parameters
params = [SYS.m; SYS.l; SYS.c];
dt = SYS.dt;


% Compute system inputs
% alpha = simpleTraj(q);

if contactMode == 2 || contactMode == 3
    currTrajPos = SYS.trajectoryPos;
    currTrajTime = SYS.trajectoryTime;
    goal_angle = getAngle(currTrajPos,currTrajTime,q(1));
    alpha = [0;goal_angle];
else
    alpha = zeros(3,1);
end


inputs = positionPID(q,dq,dt,alpha);
inputs = [0;inputs(2:end)];


% Compute EOM matrices
[M, C, N, Y] = AcroBOTDynamics(q,dq,inputs,params);


% Compute ddq
ddq = M\(Y - C*dq - N);


% Time derivative of generalized positions are the generalized velocities
dx(1:3) = dq;

% Time derivative of generalized velocities are accelerations (from EOM)
dx(4:6) = ddq;

end