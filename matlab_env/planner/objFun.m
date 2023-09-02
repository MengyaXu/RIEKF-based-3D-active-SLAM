function obj = objFun(u0)
load data range Q P x i visble M vis_feat
load sigma sigma_r

[P_pre, x_pre] = predict(2, u0, Q, P, x, zeros(6, 1));

    d = 0;
    sigma_Feat  =[];
    H_feat = {};
    for jj=1:1:M
        if visble(jj, i-1) == 1 && ismember(jj, vis_feat)
            j = find(vis_feat == jj);

            rot = R(x_pre(3),'y',0)*R(x_pre(2),'p',0)*R(x_pre(1),'r',0);
            delta = x_pre(6+3*j-2:6+3*j) - x_pre(4:6);
    
            Rr = R(x_pre(3),'y',0)*R(x_pre(2),'p',0)*R(x_pre(1),'r',1);
            Rp = R(x_pre(3),'y',0)*R(x_pre(2),'p',1)*R(x_pre(1),'r',0);
            Ry = R(x_pre(3),'y',1)*R(x_pre(2),'p',0)*R(x_pre(1),'r',0);
            Hij = [Rr'*delta Rp'*delta Ry'*delta, -rot', rot'];
            H_feat{i, jj} = Hij;
        end
    end
    k = sum(visble(:, i-1));
    N_ = diag(repmat(sigma_r^2, 3*k, 1));
        [K, H] = KF_gain(P_pre, N_, visble(:, i-1), M, H_feat, i, vis_feat);
        PX=(eye(size(P_pre,1))-K*H)*P_pre;
%         obj = det(PX);
        obj = trace(PX);
%     end
% end
end

