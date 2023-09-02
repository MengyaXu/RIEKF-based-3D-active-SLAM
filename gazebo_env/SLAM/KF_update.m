function [x,P]=KF_update(x_pre,K,z,H,P_pre,z_est, z_new_est, sigma_feat2)
Z=z;
ze = z_est;
dx = K*(Z-ze);
if isempty(dx)
    x = x_pre;
else
    x=x_pre+dx;
end
x(1) = wrapToPi(x(1));
x(2) = wrapToPi(x(2));
x(3) = wrapToPi(x(3));

P=(eye(size(P_pre,1))-K*H)*P_pre;

r=R(x(3),'y',0)*R(x(2),'p',0)*R(x(1),'r',0);
x_length = length(x);
k_new = length(z_new_est) / 3;
xf = repmat(x(4:6), k_new, 1) + reshape(r *reshape(z_new_est, 3, k_new), 3 * k_new, 1);
x = [x; xf];

KK = eye(length(x));
for i = 1:k_new
    KK(x_length+3*i-2:x_length+3*i, 4:6) = eye(3);
    KK(x_length+3*i-2:x_length+3*i, x_length+3*i-2:x_length+3*i) = r;
end
tempKK = diag(repmat(sigma_feat2, k_new, 1));
Sigma = blkdiag(P, tempKK);
P = KK * Sigma * KK';
end