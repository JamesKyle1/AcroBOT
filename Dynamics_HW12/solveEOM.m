function ddq = solveEOM(t,x,params)
% Split up state vector into generalized coordinates and velocities
q = x(1:3);
dq = x(4:6);


% Compute system inputs
alpha = [0;0;0];
% inputs = positionPID(q,dq,t,alpha);
% inputs = [0;inputs(2:end)];
inputs = [0;0;0];

% Compute EOM matrices
[M, C, N, Y] = AcroBOTDynamics(q,dq,inputs,params);

% Compute ddq
ddq = M\(Y - C*dq - N);

end