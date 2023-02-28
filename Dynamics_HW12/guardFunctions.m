function [constraintFcns,isterminal,direction] = guardFunctions(t,x,contactMode)
% Split up state vector into generalized coordinates and velocities
q = x(1:2);
dq = x(3:4);

% Compute constraint function (transition if goes negative)
a = compute_a(q);
a = a(setdiff([1:length(a)], contactMode));

% Solve the equations of motion to compute lambda (transition if goes
% positive)
ddq = solveEOM(x,contactMode);

constraintFcns = 0; % The value that we want to be zero
isterminal = 0;  % Halt integration z
direction = 0;   % The zero can be approached from either direction