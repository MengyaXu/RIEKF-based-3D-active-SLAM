function vis = visibility(feature_i,pose, j)
load dataFeature2 id1 tri1 
i1 = find(id1 == j);
% i2 = find(id2 == j);
if i1
    [k1, ~] = find(tri1 == i1);
    tri1(k1, :) = [];
end
% if i2
%     [k2, ~] = find(tri2 == i2);
%     tri2(k2, :) = [];
% end
s = feature_i(:, j) - pose;
f = zeros(3, 3);
vis = 1;
for i = 1:length(tri1(:, 1))
    for j1 = 1:3
        id = id1(tri1(i, j1));
        f(:, j1) = feature_i(:, id);
    end
%     plot3(f(1, :), f(2, :), f(3, :));
%     hold on;
%     plot3([pose(1) feature_i(1, j)], [pose(2) feature_i(2, j)], [pose(3) feature_i(3, j)]);
    e1 = plucker(f(:, 2), f(:, 1));
    e2 = plucker(f(:, 3), f(:, 2));
    e3 = plucker(f(:, 1), f(:, 3));
    L = plucker(pose, feature_i(:, j));
    s1 = side(L, e1);
    s2 = side(L, e2);
    s3 = side(L, e3);
    if (s1>0 && s2>0 && s3>0) || (s1<0 && s2<0 && s3<0)
        L2 = plucker(f(:, 1), feature_i(:, j));
        L3 = plucker(pose, f(:, 1));
        L4 = plucker(f(:, 2), f(:, 3));
        S1 = side(L4, L3);
        S2 = side(L4, L2);
        if (S1<0 && S2<0) || (S1>0 && S2>0)
            vis = 0;
            return;
        end
    end
end
end

