function [grph, vtx, unknown, occ, free, map] = newMap(op, grph, vtx, map, pose, unknown, occ, free, ranges, angles,id_f, range)
if op == 1
    load(['dataFeature' num2str(id_f)], 'goalx','goaly','goalz');
    if ~isempty(occ)
        id_occ = [];
        for id_un = 1:length(occ(1, :))
            id = occ(:, id_un);
            goal = [goalx(id(1)); goaly(id(2)); goalz(1)];
            rot = R(pose(3),'y',0)*R(pose(2),'p',0)*R(pose(1),'r',0);
            delta = goal - pose(4:6);

            y = rot' * delta;
            an = atan2(y(2),y(1));
            q = sqrt(y' * y);
            if q <= range(2) && abs(an) < pi/3 && y(1) > 0
                da = abs(angles - an);
                [angle_min, angle_id] = min(da);
                id1 = max(1, angle_id - 5);
                id2 = min(length(angles), angle_id+5);
                rg = ranges(id1:id2);

                if min(rg) > q
                    free = [free, id];
                    map(id(1), id(2)) = 0;
                    id_occ = [id_occ, id_un];
                end
            end
        end
        occ(:, id_occ) = [];
    end
    
    id_kn = [];
    for id_un = 1:length(unknown(1, :))
        id = unknown(:, id_un);
        goal = [goalx(id(1)); goaly(id(2)); goalz(1)];
        rot = R(pose(3),'y',0)*R(pose(2),'p',0)*R(pose(1),'r',0);
        delta = goal - pose(4:6);

        y = rot' * delta;
        an = atan2(y(2),y(1));
        q = sqrt(y' * y);
        if q <= range(2) && abs(an) < pi/3 && y(1) > 0
            da = abs(angles - an);
            [angle_min, angle_id] = min(da);
            id1 = max(1, angle_id - 5);
            id2 = min(length(angles), angle_id+5);
            rg = ranges(id1:id2);
            if min(rg) <= q 
                if q - min(rg) < 0.4
                    occ = [occ, id];
                    map(id(1), id(2)) = 1;
                    id_kn = [id_kn, id_un];
                end
            else
                free = [free, id];
                map(id(1), id(2)) = 0;
                id_kn = [id_kn, id_un];
            end
        end
    end
    unknown(:, id_kn) = [];
else
    [grph, vtx] = getGraph(grph, vtx, map, occ);
end
end