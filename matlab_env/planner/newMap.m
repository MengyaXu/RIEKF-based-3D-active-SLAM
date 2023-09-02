function [grph, vtx, unknown, occ, free, map] = newMap(grph, vtx, range, pose, unknown, occ, free, map,id_f)
load(['dataFeature' num2str(id_f)], 'goalx','goaly','goalz');
    if ~isempty(occ)
        id_occ = [];
        for id_un = 1:length(occ(1, :))
            id = occ(:, id_un);
            goal = [goalx(id(1)); goaly(id(2)); goalz(id(3))];
            y = (R(pose(3),'y',0)*R(pose(2),'p',0)*R(pose(1),'r',0))' * (goal - pose(4:6));
            q = sqrt(y' * y);
            if q <= range(2) %&& y(1) > 0
                vis = reachability(pose(4:6), goal,id_f);
                if vis == 1
                    free = [free, id];
                    map(id(1), id(2), id(3)) = 1;
                    id_occ = [id_occ, id_un];
                end
            end
        end
        occ(:, id_occ) = [];
    end
    
    id_kn = [];
    for id_un = 1:length(unknown(1, :))
        id = unknown(:, id_un);
        goal = [goalx(id(1)); goaly(id(2)); goalz(id(3))];
        y = (R(pose(3),'y',0)*R(pose(2),'p',0)*R(pose(1),'r',0))' * (goal - pose(4:6));
        q = sqrt(y' * y);
        if q <= range(2) %&& y(1) > 0
            vis = reachability(pose(4:6), goal,id_f);
            if vis == 0
                occ = [occ, id];
                map(id(1), id(2), id(3)) = -1;
            else
                free = [free, id];
                map(id(1), id(2), id(3)) = 1;
            end
            id_kn = [id_kn, id_un];
        end
    end
    unknown(:, id_kn) = [];

    [grph, vtx] = getGraph(grph, vtx, map, occ);
end