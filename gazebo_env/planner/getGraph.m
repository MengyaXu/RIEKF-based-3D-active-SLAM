function [grph, vtx] = getGraph(grph, vtx, map, occ)
vtx1 = [];
grph = [];
map1 = map;
if ~isempty(occ)
    for i = 1:length(occ(1, :))
        id_occ = occ(:, i);
        xx1 = id_occ(1)-1;
        xx2 = id_occ(1)+1;
        yy1 = id_occ(2)-1;
        yy2 = id_occ(2)+1;

        x1 = max(1, id_occ(1)-1);
        x2 = min(length(map(:, 1, 1)), id_occ(1)+1);
        y1 = max(1, id_occ(2)-1);
        y2 = min(length(map(1, :, 1)), id_occ(2)+1);
        v1 = map(x1, id_occ(2));
        v2 = map(x2, id_occ(2));
        v3 = map(id_occ(1), y1);
        v4 = map(id_occ(1), y2);
        v5 = map(x1, y1);
        v6 = map(x1, y2);
        v7 = map(x2, y1);
        v8 = map(x2, y2);
        if xx1 >= 1 && yy1 >= 1 && v1 == 0 && v3 == 0 && v5 == 0
            x11 = max(x1-1, 1);
            x12 = min(length(map(:,1)), x1+1);
            y11 = max(1, y1 - 1);
            y12 = min(length(map(1, :)), y1+1);

            if map1(x1, y1) ~= 2 && map1(x11, y1)~=2 && map1(x12, y1)~=2 && map1(x1, y11)~=2 && map1(x1, y12)~=2
                vtx1 = [vtx1, [x1; y1]];
                map1(x1, y1) = 2;
            end
        end
        if xx2 <= length(map(:, 1, 1)) && yy1 >= 1 && v2 == 0 && v3 == 0 && v7 == 0
            x11 = max(x2-1, 1);
            x12 = min(length(map(:,1)), x2+1);
            y11 = max(1, y1 - 1);
            y12 = min(length(map(1, :)), y1+1);
            if map1(x2, y1) ~= 2 && map1(x11, y1)~=2 && map1(x12, y1)~=2 && map1(x2, y11)~=2 && map1(x2, y12)~=2
                vtx1 = [vtx1, [x2; y1]];
                map1(x2, y1) = 2;
            end
        end
        if xx1 >= 1 && yy2 <= length(map(1, :, 1)) && v1 == 0 && v4 == 0 && v6 == 0
            x11 = max(x1-1, 1);
            x12 = min(length(map(:,1)), x1+1);
            y11 = max(1, y2 - 1);
            y12 = min(length(map(1, :)), y2+1);

            if map1(x1, y2) ~= 2 && map1(x11, y2)~=2 && map1(x12, y2)~=2 && map1(x1, y11)~=2 && map1(x1, y12)~=2
                vtx1 = [vtx1, [x1; y2]];
                map1(x1, y2) = 2;
            end
        end
        if xx2 <= length(map(:, 1, 1)) && yy2 <= length(map(1, :, 1)) && v2 == 0 && v4 == 0 && v8 == 0
            x11 = max(x2-1, 1);
            x12 = min(length(map(:,1)), x2+1);
            y11 = max(1, y2 - 1);
            y12 = min(length(map(1, :)), y2+1);
            if map1(x2, y2) ~= 2 && map1(x11, y2)~=2 && map1(x12, y2)~=2 && map1(x2, y11)~=2 && map1(x2, y12)~=2
                vtx1 = [vtx1, [x2; y2]];
                map1(x2, y2) = 2;
            end
        end
    end
end
if ~isempty(vtx1)
%     if isempty(vtx)
%         l1 = 0;
%     else
%         l1 = length(vtx);
%     end
    if length(vtx1(1, :)) > 1
        l2 = length(vtx1(1, :));
        for j = 1:l2-1
            for k = j+1:l2
                flag = 1;
                a = vtx1(:, j);
                b = vtx1(:, k);
                if b(1) == a(1)
                    for d = a(2):(b(2)-a(2))/abs(b(2)-a(2)):b(2)
                        v = map(a(1), d);
                        if v == 1
                            flag = 0;
                            break;
                        end
                    end
                elseif b(2) == a(2)
                    for d = a(1):(b(1)-a(1))/abs(b(1)-a(1)):b(1)
                        v = map(d, a(2));
                        if v == 1
                            flag = 0;
                            break;
                        end
                    end
                elseif abs(b(1) - a(1)) == 1
                    if abs(a(2) - b(2)) >= 2
                        dd = (b(2)-a(2))/abs(b(2)-a(2));
                        for d = a(2)+dd:dd:b(2)-dd
                            v = map(a(1), d);
                            vv = map(b(1), d);
                            if v == 1 || vv == 1
                                flag = 0;
                                break;
                            end
                        end
                    end
                elseif abs(b(2) - a(2)) == 1
                    if abs(a(1) - b(1)) >= 2
                        dd = (b(1)-a(1))/abs(b(1)-a(1));
                        for d = a(1)+dd:dd:b(1)-dd
                            v = map(d, a(2));
                            vv = map(d, b(2));
                            if v == 1 || vv == 1
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
                                v = map(d1, d2);
                                if v == 1
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
                    if j > l
                        grph{j} = [k, d];
                    else
                        grph{j} = [grph{j}; k, d];
                    end
                    if k > l
                        grph{k} = [j, d];
                    else
                        grph{k} = [grph{k}; j, d];
                    end
                else
                    l = length(grph);
                    if j > l
                        grph{j} = [];
                    end
                    if k > l
                        grph{k} = [];
                    end
                end
            end
        end
    else
            grph{1} = [];
    end
end
    vtx = vtx1;