function [K,H]=KF_gain(P_pre, N, visble, Num_feature, H_feat, i, vis_feat)
rows=[];
columns=[];
values=[];
k=0;

for jj=1:1:Num_feature
    if visble(jj) == 1 && ismember(jj, vis_feat)
        j = find(vis_feat == jj);
        k=k+1;
        rows=[rows,(3*k-2)*ones(1, 9),(3*k-1)*ones(1, 9), 3*k*ones(1,9)];
        columns=[columns,1:6,6+3*j-2:6+3*j,1:6,6+3*j-2:6+3*j,1:6,6+3*j-2:6+3*j];
        values=[values, H_feat{i, jj}(1,:), H_feat{i, jj}(2,:), H_feat{i, jj}(3,:)];
    end
end
H=sparse(rows,columns,values,3*k,size(P_pre,1));
K=P_pre*H'/(H*P_pre*H'+N);
end

