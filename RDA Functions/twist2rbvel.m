%% twist2rbvel(xi) maps the 6-vector twist xi to the 4x4 rigid body velocity matrix in homogeneous coordinates xi_hat

function xi_hat = twist2rbvel(xi)
    % TODO: construct the 4x4 rigid body velocity matrix in homogeneous
    % coordinates xi_hat from xi
    xi_hat = zeros(4, 4);

    v = xi(1:3,1);
    w = xi(4:6,1);

    w_hat = angvel2skew(w);

    xi_hat = [w_hat, v; [0 0 0 0]];

end