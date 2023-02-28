%% rbvel2twist(xi_hat) is the inverse mapping of twist2rbvel. 

function xi = rbvel2twist(xi_hat)
    % TODO: construct the 6-vector twist xi from xi_hat
    xi = zeros(6, 1);

    w_hat = xi_hat(1:3,1:3);
    v = xi_hat(1:3,4);

    w = skew2angvel(w_hat);

    xi = [v;w];

end
