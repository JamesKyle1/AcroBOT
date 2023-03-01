function [constraintFcns,isterminal,direction] = guardFunctions(t,x,contactMode)
% Split up state vector into generalized coordinates and velocities
q = x(1:2);
dq = x(3:4);


constraintFcns = 1; % The value that we want to be zero
isterminal = 1;  % Halt integration z
direction = -1;   % The zero can be approached from either direction