function [goal, goalID] = getFront(unknown, occ, free, map,id_f)
load(['dataFeature' num2str(id_f)], 'goalx','goaly','goalz');

front = [];
for  i = 1:length(free(1, :))
    cand = free(:, i);
    if cand(3) == 1
        x1 = max(1, cand(1)-1);
        v1 = map(x1, cand(2), cand(3));
        x2 = min(length(map(:, 1, 1)), cand(1)+1);
        v2 = map(x2, cand(2), cand(3));
        y1 = max(1, cand(2)-1);
        v3 = map(cand(1), y1, cand(3));
        y2 = min(length(map(1, :, 1)), cand(2)+1);
        v4 = map(cand(1), y2, cand(3));
        z1 = max(1, cand(3)-1);
        v5 = map(cand(1), cand(2), z1);
        z2 = min(length(map(1, 1, :)), cand(3)+1);
        v6 = map(cand(1), cand(2), z2);
        if v1 == 0 || v2 == 0 || v3 == 0 || v4 == 0 || v5 == 0 || v6 == 0
            [val, vec] = getValue(cand, map);
            front = [front, [cand; val; vec]];
        end
    end
end
if isempty(front)
    goal = [];
    goalID = [];
else
    vm = max(front(4, :));
    idM = find(front(4, :) == vm);
    r = randperm(length(idM));
    % idm = idM(r(1));
    idm = idM(1);
%     values = front(5, :).^2 + front(6, :).^2 + front(7, :).^2;
%     [vm, idm] = max(values);
    goalID = front(1:3, idm);
    goal = [goalx(front(1, idm)); goaly(front(2, idm)); goalz(front(3, idm))];
end

