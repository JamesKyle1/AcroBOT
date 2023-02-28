function inputs = positionPID(q,dq,t,alpha)
% This funciton will take in the dynamics of gymnastics bot and return
% a set of inputs to the system to drive the desired position
persistent integral e_prev t_prev alpha_prev;

inputs = zeros(length(q),1);

if isempty(alpha_prev)
    alpha_prev = alpha;
else alpha ~= alpha_prev
    alpha_prev = alpha;
    integral = [];
end

if isempty(e_prev)
    e_prev = zeros(3,1);
end

if isempty(t_prev)
    t_prev = 0;
end

kp = 23.2817;
ki = 13.5832;
kd = 9.9763;

T = (t-t_prev);

e = alpha - q;

if isempty(integral)
    integral = ki.*e.*T;
else
    integral = integral + ki.*e.*T;
end

if T >= 10e-10
    derivative = kd.*(e - e_prev)./T;
else
    derivative = 0;
end

proportional = kp.*e;


inputs = proportional + integral + derivative;


e_prev = e;
t_prev = t;

end