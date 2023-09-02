function j = jaco(theta)
t = norm(theta);
if t < 0.00000001
    j = eye(3);
else
    j = eye(3) - (1 - cos(t)) * skew(theta)/t^2 + (t - sin(t))*skew(theta)^2 / t^3;
end
end

