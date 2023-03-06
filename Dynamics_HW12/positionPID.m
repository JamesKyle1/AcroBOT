function inputs = positionPID(q,dq,dt,alpha)
% This funciton will take in the dynamics of gymnastics bot and return
% a set of inputs to the system to drive the desired position
persistent integral

kp = 100;
ki = 5;
kd = 10;


e = alpha - q;

proportional = kp.*e;
derivative = kd.*dq;
integral = ki.*e.*dt;


inputs = proportional - derivative + integral;


end