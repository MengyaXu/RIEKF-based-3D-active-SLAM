function obj = objFun(u0, map,id_f)
load data range Q P x i visble M vis_feat
load sigma sigma_r
load(['dataFeature' num2str(id_f)], 'goalx','goaly','goalz');

map2 = map(:,:,1);
[P_pre, x_pre] = RIEKF_predict(2, u0, Q, P, x, zeros(6, 1));
z_est = [];

xid = max(1, (x_pre(4)-goalx(1)+1) / (goalx(end)-goalx(1)+1) * length(goalx));
yid = max(1, (x_pre(5)-goaly(1)+1) / (goaly(end)-goaly(1)+1) * length(goaly));
% zid = max(1, (x_pre(6)-goalz(1)+1) / (goalz(end)-goalz(1)+1) * length(goalz));
a = [max(1, min(length(goalx),round(xid))); 
    max(1, min(length(goaly),round(yid))); 
    1];
vis = zeros(M, 1);
    H_feat = {};
for jj=1:1:M
    if ismember(jj, vis_feat)
        j = find(vis_feat == jj);

        rot = R(x_pre(3),'y',0)*R(x_pre(2),'p',0)*R(x_pre(1),'r',0);
        delta = x_pre(6+3*j-2:6+3*j) - x_pre(4:6);
        an = atan2((abs(delta(3))+0.08),max(norm(delta(1), delta(2)+0.08),norm(delta(1), delta(2)-0.08)));
        y = rot' * delta;
        q = sqrt(y' * y);
        if q <= range(2) && abs(an) < pi/4 && y(1) > 0
            flag = 1;
            yid = max(1, (x_pre(6+3*j-2)-goaly(1)+1) / (goaly(end)-goaly(1)+1) * length(goaly));
            xid = max(1, (x_pre(6+3*j-1)-goalx(1)+1) / (goalx(end)-goalx(1)+1) * length(goalx));
%             zid = max(1, (x_pre(6+3*j)-goalz(1)+1) / (goalz(end)-goalz(1)+1) * length(goalz));
            b = [max(1, min(length(goalx),round(xid))); 
                max(1, min(length(goaly),round(yid))); 
                1];
            if b(1) == a(1)
                if b(2) ~= a(2)
                    for d = a(2):(b(2)-a(2))/abs(b(2)-a(2)):b(2)
                        v = map2(a(1), d);
                        if v ~= 1
                            flag = 0;
                            break;
                        end
                    end
                end
            elseif abs(b(1) - a(1)) == 1
                if a(2) ~= b(2)
                    for d = a(2):(b(2)-a(2))/abs(b(2)-a(2)):b(2)
                        v = map2(a(1), d);
                        vv = map2(b(1), d);
                        if v == -1 || vv == -1
                            flag = 0;
                            break;
                        end
                    end
                end
            else
                kk = (b(2) - a(2)) / (b(1) - a(1));
                for d = (b(1)-a(1))/abs(b(1)-a(1)):(b(1)-a(1))/abs(b(1)-a(1)):b(1)-a(1)
                    y0 = a(2) + d*kk;
                    y1 = ceil(y0);
                    y2 = floor(y0);
                    v = map2(a(1) + d, y1);
                    v2 = map2(a(1) + d, y2);
                    if v ~= 1 || v2 ~= 1
                        flag = 0;
                        break;
                    end
                end
            end
            if flag == 1
                z_est = [z_est; y];
                vis(jj)  = 1;
                Rr = R(x_pre(3),'y',0)*R(x_pre(2),'p',0)*R(x_pre(1),'r',1);
                Rp = R(x_pre(3),'y',0)*R(x_pre(2),'p',1)*R(x_pre(1),'r',0);
                Ry = R(x_pre(3),'y',1)*R(x_pre(2),'p',0)*R(x_pre(1),'r',0);
                Hij = [Rr'*delta Rp'*delta Ry'*delta, -rot', rot'];
                H_feat{i, jj} = Hij;
            end
        end
    end
end
    k = sum(vis);
%     if k == 0
%         PX = P_pre;
%     else
        N_ = sparse(diag(z_est.^2) * sigma_r^2);
%         vis
%         H_feat
        [K, H] = KF_gain(P_pre, N_, vis, M, H_feat, i, vis_feat);
        PX=(eye(size(P_pre,1))-K*H)*P_pre;
%     end
%         obj = det(PX);
%         obj = abs(trace(PX) - trace(P));
        obj = trace(PX);
%     end
% end
end

