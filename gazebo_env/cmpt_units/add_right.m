function x = add_right(x, s)
s_theta = s(1:3);
s_p = s(4:6);
sizeS = size(s, 1);
k = (sizeS - 6) / 3;

expS = so3_exp(s_theta);

xr = expS * x(4:6) + jaco(-s_theta) * s_p;

if k >= 1
    s_l = reshape(s(7:end), 3, k);
    xf = expS * reshape(x(7:end), 3, k) + jaco(-s_theta) * s_l;
    xf = reshape(xf, 3 * k, 1);
end
r=R(x(3),'y',0)*R(x(2),'p',0)*R(x(1),'r',0);
x_R = expS * r;
theta2 = -asin(x_R(3,1));
theta1 = atan2(x_R(3, 2), x_R(3, 3));
theta3 = atan2(x_R(2, 1), x_R(1, 1));
x_theta = [theta1; theta2; theta3];
x = [x_theta; xr; xf];
end

