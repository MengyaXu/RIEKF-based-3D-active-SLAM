function [value, vector] = getfeat(x, cand, map, range)
load dataFeature2 goalx goaly goalz
for i = 7:3:length(x)-2
    f = x(i, i+1, i+2);
    y = (R(x(3),'y',0)*R(x(2),'p',0)*R(x(1),'r',0))' * (f - x(4:6));
            q = sqrt(y' * y);
            if q <= range(2) && y(1) > 0
                vis = reachability(x(4:6), f);
                if vis == 1
                    free = [free, id];
                    map(id(1), id(2), id(3)) = 1;
                    id_occ = [id_occ, id_un];
                end
            end
end
n = 2;
num_un = 0;
num_occ = 0;
D_un = [0;0;0];
D_occ = [0;0;0];
    for x = max(1, cand(1)-n):min(length(map(:, 1, 1)), cand(1)+n)
        for y = max(1, cand(2)-n):min(length(map(1, :, 1)), cand(2)+n)
            for z = max(1, cand(3)-n):min(length(map(1, 1, :)), cand(3)+n)
                sur = [x; y; z];
                    d = sur - cand(1:3);
                if map(x, y, z) == 0
                    num_un = num_un + 1;
                    D_un = D_un + d;
                end
                if map(x, y, z) == -1
                    num_occ = num_occ + 1;
                    D_occ = D_occ + d;
                end
            end
        end
    end
%     value = num_un;% - num_occ;
vector = D_un;
    value = num_un - num_occ/3;
end

