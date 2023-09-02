function [goal, goalID] = getacc(x, unknown, occ, free, map, range)
load dataFeature2 goalx goaly goalz
front = [];
map2 = map(:, :, 2);
for  i = 1:length(free(1, :))
    cand = free(:, i);
    if cand(3) == 2
        [val, vec] = getfeat(x, cand, map, range);
        front = [front, [cand; val; vec]];
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

