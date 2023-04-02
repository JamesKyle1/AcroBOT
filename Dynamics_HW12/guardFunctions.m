function [constraintFcns,isterminal,direction] = guardFunctions(t,x,contactMode,hollowRegion,archRegion)
% Split up state vector into generalized coordinates and velocities
q = x(1:3);
dq = x(4:6);

backVelTol = 2;

constraintFcns = [dq(1) - backVelTol; ones(2,1).*(2*pi + q(1)) - archRegion; ones(2,1).*(2*pi + q(1)) - hollowRegion]; % The value that we want to be zero
isterminal = ones(length(constraintFcns),1);  % Halt integration z
direction = [1; -ones(length(constraintFcns)-1,1)];   % The zero can be approached from either direction