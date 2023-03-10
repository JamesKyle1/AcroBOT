function inputs = positionPID(q,dq,dt,alpha)
% This funciton will take in the dynamics of gymnastics bot and return
% a set of inputs to the system to drive the desired position
persistent integral prev_alpha

if isempty(prev_alpha)
    prev_alpha = alpha;
else
    if prev_alpha ~= alpha
        prev_alpha = alpha;
        integral = zeros(3,1);
    end
end

kp = [0;10;10];
ki = [0;0.1;0.1];
kd = [0;0.5;0.5];


e = alpha - q;

proportional = kp.*e;
derivative = kd.*dq;
integral = ki.*e.*dt;


inputs = proportional - derivative + integral;

for i = 1:3
    if inputs(i) > 1
        inputs(i) = 1;
    elseif inputs(i) < -1
        inputs(i) = -1;
    end
end


end