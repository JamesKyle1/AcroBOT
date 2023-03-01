function ddq = solveEOM(t,x,dt,params)
% Split up state vector into generalized coordinates and velocities
q = x(1:3);
dq = x(4:6);
% de = x(7:9);
% dde = x(10:12);

% Compute system inputs
alpha = [0;-pi/6;-pi/6];
inputs = positionPID(q,dq,dt,alpha);
inputs = [0;inputs(2:end)];

% Compute EOM matrices
[M, C, N, Y] = AcroBOTDynamics(q,dq,inputs,params);

% Compute ddq
ddq = M\(Y - C*dq - N);

end