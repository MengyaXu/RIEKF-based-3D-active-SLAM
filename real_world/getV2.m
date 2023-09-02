function [v, u] = getV2(alg, op, x, i, goal, explore, map, lock,id_f, ranges, angles)
if op == 1
%     yell = 0;
    yell = pi/12;
        u = [0; 0; yell; -0.1; 0; 0];
        r=R(yell,'y',0)*R(0,'p',0)*R(0,'r',0);
        vxyz = r * [-0.1; 0; 0];
%         v = u;
        v = [0; 0; yell; vxyz];
elseif op == 2
    
    vmin = Inf;
%     rr = -pi/6:pi/6:pi/6;
%     rp = -pi/6:pi/6:pi/6;
    ry = -pi/3:pi/12:pi/3;
    dx = 0:0.06:0.2;
%     dy = -2*l:1*l/2:2*l;
%     dz = -0.5:0.5:0.5;
    
    count = 0;
    Xr = [];
    V = [];
    U = [];

    for yell = 1:length(ry)
        dyell = abs(angles - ry(yell));
        [angle_min, angle_id] = min(dyell);
        id1 = max(1, angle_id - 60);
        id2 = min(length(angles), angle_id+60);
        rg = ranges(id1:id2);
        for i = 1:length(dx)
            dxx = rg - dx(i);
            dmin = min(dxx);
            if dmin > 0.15/cos(ry(yell)) || dx(i) == 0
                count = count + 1;
                r=R(ry(yell),'y',0)*R(0,'p',0)*R(0,'r',0);
                vxyz = r * [dx(i); 0; 0];
                v0 = [0; 0; ry(yell); vxyz];
                if v0 == 0
                    break;
                end
                
                V = [V v0];
                u0 = [0; 0; ry(yell); dx(i); 0; 0];
                U = [U u0];
                rx = R(x(3),'y',0)*R(x(2),'p',0)*R(x(1),'r',0);
                motion = [0;0; ry(yell); rx * vxyz];
                xr_pre = x(1:6) + motion;
                Xr = [Xr xr_pre];
            else
                yell
                dx(i)
            end
        end
    end
    
%     tic;
%     [V_f, U_f] = candidate(V, U, Xr, goal, explore,id_f);

a = Xr(4:5, :) - goal;
d_goal = sqrt(a(1,:).^2 + a(2, :).^2);
[d_g, id_g] = sort(d_goal);

l = length(Xr(1, :));
if explore == 0
    V_f = V(:, id_g(1: ceil(l/9)));
    U_f = U(:, id_g(1: ceil(l/9)));
    Xr_f = Xr(:, id_g(1: ceil(l/9)));
else
    V_f = V(:, id_g(1));
    U_f = U(:, id_g(1));
    Xr_f = Xr(:, id_g(1));
end
%     toc;
%     t1 = toc
%     length(V_f(1, :))
    
%     tic;
% u = [];
% v= [];
id = 1;
    for i = 1:length(V_f(1, :))
        if alg == 1
            obj = objFun(V_f(:, i), map,id_f);
        elseif alg == 2
            obj = RIobjFun(V_f(:, i), map,id_f);
        end
        if obj < vmin
            vmin = obj;
            v = V_f(:, i);
            u = U_f(:,i);
            id = i;
        end
    end
%     if isempty(u) || isempty(v)
%         u = [];
%     end
%     figure(1)
%     plot3(Xr(4, :), Xr(5, :), Xr(6, :), 'g*');
% plot3(Xr_f(4, :), Xr_f(5, :), Xr_f(6, :), 'g*');
end

