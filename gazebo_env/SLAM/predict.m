function [P_pre,x_pre]=predict(op, u, Q, P, x, noise_odem)
r=R(x(3),'y',0)*R(x(2),'p',0)*R(x(1),'r',0);
k = length(x(7:end)) / 3;
motion = [u(1:3);r * u(4:6); zeros(3*k, 1)];
if op == 1  % actual pose
    x_pre = x + motion(1:6);
    P_pre = [];
elseif op == 2  % predict vector
    x_pre = x + motion + [noise_odem; zeros(3*k, 1)];
    
    Rr = R(x(3),'y',0)*R(x(2),'p',0)*R(x(1),'r',1);
    Rp = R(x(3),'y',0)*R(x(2),'p',1)*R(x(1),'r',0);
    Ry = R(x(3),'y',1)*R(x(2),'p',0)*R(x(1),'r',0);
    Fx = [eye(3,3) zeros(3,3);
        Rr*u(4:6) Rp*u(4:6) Ry*u(4:6)  eye(3)];
    F = sparse(blkdiag(Fx, eye(3*k)));
    G = sparse([blkdiag(eye(3,3), r); zeros(3*k, 6)]);
    P_pre=F*P*F' + G*Q*G';
end
x_pre(1) = wrapToPi(x_pre(1));
x_pre(2) = wrapToPi(x_pre(2));
x_pre(3) = wrapToPi(x_pre(3));