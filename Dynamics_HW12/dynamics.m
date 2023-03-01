function dx = dynamics(t,x,SYS)
% Initialize time derivative of state vector as column vector
dx = zeros(length(x),1);

% Split up state vector into generalized coordinates and velocities
q = x(1:3);
dq = x(4:6);


% System Parameters
params = [SYS.m; SYS.l; SYS.c];
dt = SYS.dt;


% Compute system inputs
archRegion = [-pi/4;-5*pi/12];
hollowRegion = [-7*pi/12;-3*pi/4];
alpha = pi/4;
if q(1) - archRegion(1) < 0 && q(1) - archRegion(2) > 0
    alpha = [0;alpha;alpha];
elseif q(1) - hollowRegion(1) < 0 && q(1) - hollowRegion(2) > 0
    alpha = -[0;alpha;alpha];
else
    alpha = [0;0;0];
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