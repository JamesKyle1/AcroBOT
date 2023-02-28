%% tform2adjoint(g) maps the the rigid body transformation g, 
%% in homogeneous coordinates, to the transformation adjoint matrix, Adg.

function Adg = tform2adjoint(g)
    % TODO: construct the 6x6 transformation adjoint matrix Adg from g
    if length(g(:,1)) == 4
        Adg = zeros(6, 6);
    
        R = g(1:3,1:3);
        p = g(1:3,4);
    
        p_hat = angvel2skew(p);
    
        Adg = [     R       p_hat*R ;
                zeros(3,3)     R    ];
    elseif length(g(:,1)) == 3
        Adg = zeros(3, 3);

        p = g(1:2,3);
    
        p_new = [p(2) -p(1) 1]';
    
        Adg = [g(1:3,1:2) p_new];
    end

end
