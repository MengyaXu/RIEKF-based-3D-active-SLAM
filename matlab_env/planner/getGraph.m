function [grph, vtx] = getGraph(grph, vtx, map, occ)
map2 = map(:, :, 1);
vtx1 = [];
if ~isempty(occ)
for i = 1:length(occ(1, :))
    id_occ = occ(:, i);
    if id_occ(3) == 1
        xx1 = id_occ(1)-1;
        xx2 = id_occ(1)+1;
        yy1 = id_occ(2)-1;
        yy2 = id_occ(2)+1;

        x1 = max(1, id_occ(1)-1);
        x2 = min(length(map2(:, 1, 1)), id_occ(1)+1);
        y1 = max(1, id_occ(2)-1);
        y2 = min(length(map2(1, :, 1)), id_occ(2)+1);
        v1 = map2(x1, id_occ(2));
        v2 = map2(x2, id_occ(2));
        v3 = map2(id_occ(1), y1);
        v4 = map2(id_occ(1), y2);
        v5 = map2(x1, y1);
        v6 = map2(x1, y2);
        v7 = map2(x2, y1);
        v8 = map2(x2, y2);
        if xx1 >= 1 && yy1 >= 1 && v1 == 1 && v3 == 1 && v5 == 1
            d = 0;
            if ~isempty(vtx)
                a = (vtx(1, :) == x1);
                b = (vtx(2, :) == y1);
                c = (a == b);
                if sum(a) ~= 0 && sum(b) ~= 0
                    d = sum(c);
                end
            end
            if ~isempty(vtx1)
                a = (vtx1(1, :) == x1);
                b = (vtx1(2, :) == y1);
                c = (a == b);
                if sum(a) ~= 0 && sum(b) ~= 0
                    d = d + sum(c);
                end
            end
            if d == 0
                vtx1 = [vtx1, [x1; y1]];
            end
        end
        if xx2 <= length(map2(:, 1, 1)) && yy1 >= 1 && v2 == 1 && v3 == 1 && v7 == 1
            d = 0;
            if ~isempty(vtx)
                a = (vtx(1, :) == x2);
                b = (vtx(2, :) == y1);
                c = (a == b);
                if sum(a) ~= 0 && sum(b) ~= 0
                    d = sum(c);
                end
            end
            if ~isempty(vtx1)
                a = (vtx1(1, :) == x2);
                b = (vtx1(2, :) == y1);
                c = (a == b);
                if sum(a) ~= 0 && sum(b) ~= 0
                    d = d + sum(c);
                end
            end
            if d == 0
                vtx1 = [vtx1, [x2; y1]];
            end
        end
        if xx1 >= 1 && yy2 <= length(map2(1, :, 1)) && v1 == 1 && v4 == 1 && v6 == 1
            d = 0;
            if ~isempty(vtx)
                a = (vtx(1, :) == x1);
                b = (vtx(2, :) == y2);
                c = (a == b);
                if sum(a) ~= 0 && sum(b) ~= 0
                    d = sum(c);
                end
            end
            if ~isempty(vtx1)
                a = (vtx1(1, :) == x1);
                b = (vtx1(2, :) == y2);
                c = (a == b);
                if sum(a) ~= 0 && sum(b) ~= 0
                    d = d + sum(c);
                end
            end
            if d == 0
                vtx1 = [vtx1, [x1; y2]];
            end
        end
        if xx2 <= length(map2(:, 1, 1)) && yy2 <= length(map2(1, :, 1)) && v2 == 1 && v4 == 1 && v8 == 1
            d = 0;
            if ~isempty(vtx)
                a = (vtx(1, :) == x2);
                b = (vtx(2, :) == y2);
                c = (a == b);
                if sum(a) ~= 0 && sum(b) ~= 0
                    d = sum(c);
                end
            end
            if ~isempty(vtx1)
                a = (vtx1(1, :) == x2);
                b = (vtx1(2, :) == y2);
                c = (a == b);
                if sum(a) ~= 0 && sum(b) ~= 0
                    d = d + sum(c);
                end
            end
            if d == 0
                vtx1 = [vtx1, [x2; y2]];
            end
        end
    end
end
end
if ~isempty(vtx) && ~isempty(vtx1)
    l1 = length(vtx(1, :));
    l2 = length(vtx1(1, :));
    for j = 1:l1
        for k = 1:l2
            flag = 1;
            a = vtx(:, j);
            b = vtx1(:, k);
                if b(1) == a(1)
                    for d = a(2):(b(2)-a(2))/abs(b(2)-a(2)):b(2)
                        v = map2(a(1), d);
                        if v == -1
                            flag = 0;
                            break;
                        end
                    end
                elseif b(2) == a(2)
                    for d = a(1):(b(1)-a(1))/abs(b(1)-a(1)):b(1)
                        v = map2(d, a(2));
                        if v == -1
                            flag = 0;
                            break;
                        end
                    end
                elseif abs(b(1) - a(1)) == 1
                    if abs(a(2) - b(2)) >= 2
                        dd = (b(2)-a(2))/abs(b(2)-a(2));
                        for d = a(2)+dd:dd:b(2)-dd
                            v = map2(a(1), d);
                            vv = map2(b(1), d);
                            if v == -1 || vv == -1
                                flag = 0;
                                break;
                            end
                        end
                    end
                elseif abs(b(2) - a(2)) == 1
                    if abs(a(1) - b(1)) >= 2
                        dd = (b(1)-a(1))/abs(b(1)-a(1));
                        for d = a(1)+dd:dd:b(1)-dd
                            v = map2(d, a(2));
                            vv = map2(d, b(2));
                            if v == -1 || vv == -1
                                flag = 0;
                                break;
                            end
                        end
                    end
                elseif abs(b(1) - a(1)) >= 2
                    if abs(a(2) - b(2)) >= 2
                        dd1 = (b(1)-a(1))/abs(b(1)-a(1));
                        dd2 = (b(2)-a(2))/abs(b(2)-a(2));
                        for d1 = a(1)+dd1:dd1:b(1)-dd1
                            for d2 = a(2)+dd2:dd2:b(2)-dd2
                                v = map2(d1, d2);
                                if v == -1
                                    flag = 0;
                                    break;
                                end
                            end
                        end
                    end
                end
            if  flag == 1
                d = norm(a-b);
                grph{j} = [grph{j}; l1+k, d];
                l = length(grph);
                if l1+k > l
                    grph{l1+k} = [j, d];
                else
                    grph{l1+k} = [grph{l1+k}; j, d];
                end
            end
        end
    end
end
if ~isempty(vtx1)
    if isempty(vtx)
        l1 = 0;
    else
        l1 = length(vtx);
    end
    if length(vtx1(1, :)) > 1
        l2 = length(vtx1(1, :));
        for j = 1:l2-1
            for k = j+1:l2
                flag = 1;
                a = vtx1(:, j);
                b = vtx1(:, k);
                if b(1) == a(1)
                    for d = a(2):(b(2)-a(2))/abs(b(2)-a(2)):b(2)
                        v = map2(a(1), d);
                        if v == -1
                            flag = 0;
                            break;
                        end
                    end
                elseif b(2) == a(2)
                    for d = a(1):(b(1)-a(1))/abs(b(1)-a(1)):b(1)
                        v = map2(d, a(2));
                        if v == -1
                            flag = 0;
                            break;
                        end
                    end
                elseif abs(b(1) - a(1)) == 1
                    if abs(a(2) - b(2)) >= 2
                        dd = (b(2)-a(2))/abs(b(2)-a(2));
                        for d = a(2)+dd:dd:b(2)-dd
                            v = map2(a(1), d);
                            vv = map2(b(1), d);
                            if v == -1 || vv == -1
                                flag = 0;
                                break;
                            end
                        end
                    end
                elseif abs(b(2) - a(2)) == 1
                    if abs(a(1) - b(1)) >= 2
                        dd = (b(1)-a(1))/abs(b(1)-a(1));
                        for d = a(1)+dd:dd:b(1)-dd
                            v = map2(d, a(2));
                            vv = map2(d, b(2));
                            if v == -1 || vv == -1
                                flag = 0;
                                break;
                            end
                        end
                    end
                elseif abs(b(1) - a(1)) >= 2
                    if abs(a(2) - b(2)) >= 2
                        dd1 = (b(1)-a(1))/abs(b(1)-a(1));
                        dd2 = (b(2)-a(2))/abs(b(2)-a(2));
                        for d1 = a(1)+dd1:dd1:b(1)-dd1
                            for d2 = a(2)+dd2:dd2:b(2)-dd2
                                v = map2(d1, d2);
                                if v == -1
                                    flag = 0;
                                    break;
                                end
                            end
                        end
                    end
                end
                if  flag == 1
                    d = norm(a-b);
                    l = length(grph);
                    if l1+j > l
                        grph{l1+j} = [l1+k, d];
                    else
                        grph{l1+j} = [grph{l1+j}; l1+k, d];
                    end
                    if l1+k > l
                        grph{l1+k} = [l1+j, d];
                    else
                        grph{l1+k} = [grph{l1+k}; l1+j, d];
                    end
                else
                    l = length(grph);
                    if l1+j > l
                        grph{l1+j} = [];
                    end
                    if l1+k > l
                        grph{l1+k} = [];
                    end
                end
            end
        end
    else
        l = length(grph);
        if l1+1 > l
            grph{l1+1} = [];
        end
    end
end
    vtx = [vtx, vtx1];