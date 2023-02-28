function C = coriolisMatrix(M,q,dq)

C  = sym(zeros(2, 2));

n = length(q);

for i = 1:n
    for j = 1:n
        sum = 0;
        for k = 1:n
            sum = sum + (diff(M(i,j),q(k)) + diff(M(i,k),q(j)) - diff(M(j,k),q(i)))*dq(k);
        end
        C(i,j) = 1/2*sum;
    end
end

end