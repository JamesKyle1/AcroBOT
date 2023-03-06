function alpha = getAngle(trajectoryPosition, trajectoryTiming, q)

n = length(trajectoryPosition(:,1));
alpha = zeros(n,1);

for i = 1:n
    foo = trajectoryPosition(trajectoryTiming(i,:) > q);
    alpha(i) = foo(end);
end

end