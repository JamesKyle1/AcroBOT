function alpha = simpleTraj(q)

archRegion = [-pi/4;-5*pi/12];
hollowRegion = [-7*pi/12;-3*pi/4];

goalAngle = pi/4;

if q(1) - archRegion(1) < 0 && q(1) - archRegion(2) > 0
    alpha = [0;goalAngle;goalAngle];
elseif q(1) - hollowRegion(1) < 0 && q(1) - hollowRegion(2) > 0
    alpha = -[0;goalAngle;goalAngle];
else
    alpha = [0;0;0];
end


end