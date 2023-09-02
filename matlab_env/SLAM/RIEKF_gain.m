function [K,H]=RIEKF_gain(x_pre, P_pre, N, visble, M, vis_feat, k)

% k = sum(visble);
r=R(x_pre(3),'y',0)*R(x_pre(2),'p',0)*R(x_pre(1),'r',0);
H2 = zeros(3*k, length(x_pre));

% temp = repmat({eye(3)}, Num_feature, 1);
% Rr = blkdiag(temp{:});
j = 0;
for jj = 1:M
    if visble(jj) == 1 && ismember(jj, vis_feat)
        j = j + 1;
        id = find(vis_feat == jj);
        H2(3*j-2 : 3*j, 4:6) = -r';
        H2(3*j-2 : 3*j, 6+3*id-2:6+3*id) = r';
    end
end
% H = H1 .* H2;
H = H2;
K = P_pre*H'/(H*P_pre*H'+N);
end

