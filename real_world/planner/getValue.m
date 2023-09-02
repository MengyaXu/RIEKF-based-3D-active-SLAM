function [value, vector] = getValue(cand, map)
n = 2;
num_un = 0;
num_occ = 0;
D_un = [0;0];
D_occ = [0;0];
    for x = max(1, cand(1)-n):min(length(map(:, 1, 1)), cand(1)+n)
        for y = max(1, cand(2)-n):min(length(map(1, :, 1)), cand(2)+n)
%             for z = max(1, cand(3)-n):min(length(map(1, 1, :)), cand(3)+n)
                sur = [x; y];
                    d = sur - cand(1:2);
                if map(x, y) == -1
                    num_un = num_un + 1;
                    D_un = D_un + d;
                end
                if map(x, y) == 1
                    num_occ = num_occ + 1;
                    D_occ = D_occ + d;
                end
%             end
        end
    end
%     value = num_un;% - num_occ;
vector = D_un;
    value = num_un - num_occ/3;
end

