function [P_pre,x_pre]=RIEKF_predict(op, u, Q, P, x, noise_odem)
noise_odem(1:2) = 0;
noise_odem(6) = 0;
r=R(x(3),'y',0)*R(x(2),'p',0)*R(x(1),'r',0);
k = length(x(7:end)) / 3;
motion = [u(1:3);r * u(4:6); zeros(3*k, 1)];
if op == 1  % actual pose
%     x_pre = x + motion(1:6) + noise_odem;
    x_pre = x + motion(1:6);
    P_pre = [];
elseif op == 2  % predict vector
    x_pre = x + motion + [noise_odem; zeros(3*k, 1)];
%     x_pre = x + motion;
    
%     Rr = R(x(3),'y',0)*R(x(2),'p',0)*R(x(1),'r',1);
%     Rp = R(x(3),'y',0)*R(x(2),'p',1)*R(x(1),'r',0);
%     Ry = R(x(3),'y',1)*R(x(2),'p',0)*R(x(1),'r',0);
    F = sparse(eye(length(x)));

    j = jaco(-u(1:3));
%     e = so3_exp(-u(1:3));
    temp = repmat({r}, 1, k+2);
    A = blkdiag(temp{:});
    A(4:6, 1:3) = skew(x(4:6)) * r;

    for i = 1:k
        A(6 + 3*i - 2 : 6 + 3*i, 1:3) = skew(x(6 + 3*i - 2 : 6 + 3*i)) * r;
    end

    B1 = [-j, zeros(3);
        -skew(u(4:6)) * j, -eye(3)];
    B = [B1; sparse(3*k, 6)];
    B = sparse(B);

    adA = A*B;

%     Gx = [Rr*u(4:6) Rp*u(4:6) Ry*u(4:6)  zeros(3);
%         zeros(3, 6);
%         ];
%     F = sparse(blkdiag(Fx, eye(3*k)));
%     G = sparse(blkdiag(eye(3,3), r, zeros(3*k)));
%     G = sparse(blkdiag(eye(3,3), r, zeros(3*k)));
    P_pre=F*P*F' + adA*Q*adA';
end
x_pre(1) = wrapToPi(x_pre(1));
x_pre(2) = wrapToPi(x_pre(2));
x_pre(3) = wrapToPi(x_pre(3));