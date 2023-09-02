function goals = getpath2(goalID, pose, grph, vtx, map, free,id_f)
load(['dataFeature' num2str(id_f)], 'goalx','goaly','goalz');

% map_ = (map(:,:,2) ~= map(:,:,1) | map(:,:,2) ~= map(:,:,3));
% map2 = map(:, :, 2) + map_;
map2 = map(:, :, 1);
free2id = free(3, :) == 1;
free2 = free(:, free2id);
freex = goalx(free2(1,:));
freey = goaly(free2(2,:));
freez = goalz(free2(3,:));
d = [freex;freey] - pose(1:2);
ed = sqrt(d(1,:).^2 + d(2, :).^2);
[~, aid] = min(ed);
p = free2(:, aid);
a = p;

if goalID(3) == 1
    g = goalID;
elseif map2(goalID(1),goalID(2)) == 1
    g = goalID;
else
    d = free2(1:2, :) - goalID(1:2);
    ed = sqrt(d(1,:).^2 + d(2, :).^2);
    [~, bid] = min(ed);
    g = free2(:, bid);
end
b = g;

% xid = max(1, (pose(1)-goalx(1)+1) / (goalx(end)-goalx(1)+1) * length(goalx));
% yid = max(1, (pose(2)-goaly(1)+1) / (goaly(end)-goaly(1)+1) * length(goaly));
% zid = max(1, (pose(3)-goalz(1)+1) / (goalz(end)-goalz(1)+1) * length(goalz));
% p1 = [max(1, min(length(goalx),round(xid))); 
%     max(1, min(length(goaly),round(yid))); 
%     max(1, min(length(goalz),round(zid)))];
% 
    flag = 1;
%     a = p1;
%     b = goalID;
    if b(1) == a(1)
        if b(2) ~= a(2)
            for d = a(2):(b(2)-a(2))/abs(b(2)-a(2)):b(2)
                v = map2(a(1), d);
                if v ~= 1
                    flag = 0;
                    break;
                end
            end
        end
    elseif b(2) == a(2)
        if b(1) ~= a(1)
            for d = a(1):(b(1)-a(1))/abs(b(1)-a(1)):b(1)
                v = map2(d, a(2));
                if v ~= 1
                    flag = 0;
                    break;
                end
            end
        end
    elseif abs(b(1) - a(1)) == 1
        if abs(a(2) - b(2)) >= 2
            dd = (b(2)-a(2))/abs(b(2)-a(2));
            for d = a(2)+dd:dd:b(2)-dd
                v = map2(a(1), d);
                vv = map2(b(1), d);
                if v ~= 1 || vv ~= 1
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
                if v ~= 1 || vv ~= 1
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
                    if v ~= 1
                        flag = 0;
                        break;
                    end
                end
            end
        end
    end
    if  flag == 1
        id = [a, goalID];
        goals = [goalx(id(1, :)); goaly(id(2, :)); goalz(id(3, :))];
        return;
    end

% a = vtx - p1(1:2);
% ed = sqrt(a(1,:).^2 + a(2, :).^2);
% [~, pid] = min(ed);
% p = vtx(:, pid);
% 
% b = vtx - goalID(1:2);
% ged = sqrt(b(1,:).^2 + b(2, :).^2);
% [~, gID] = min(ged);
% id = [[p;2], goalID];
vtx1 = [a(1:2),b(1:2)];
if ~isempty(vtx)
    l1 = length(vtx(1, :));
    for j = 1:l1
        for k = 1:2
            flag = 1;
            a = vtx(:, j);
            b = vtx1(1:2, k);
            if b(1) == a(1) && b(2) ~= a(2)
                for d = a(2):(b(2)-a(2))/abs(b(2)-a(2)):b(2)
                    v = map2(a(1), d);
                    if v ~= 1
                        flag = 0;
                        break;
                    end
                end
            elseif b(2) == a(2) && b(1) ~= a(1)
                for d = a(1):(b(1)-a(1))/abs(b(1)-a(1)):b(1)
                    v = map2(d, a(2));
                    if v ~= 1
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
                        if (v == -1 || vv == -1)||(v ~= 1 && vv ~= 1)
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
                        if (v == -1 || vv == -1)||(v ~= 1 && vv ~= 1)
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
                            if v ~= 1
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
        pid = length(vtx(1,:))+1;
        if pid > length(grph)
            a = vtx - p(1:2);
            ed = sqrt(a(1,:).^2 + a(2, :).^2);
            [~, pid] = min(ed);
            goali = [pid; pid];
        else
            if isempty(grph{pid})
                a = vtx - p(1:2);
                ed = sqrt(a(1,:).^2 + a(2, :).^2);
                [~, pid] = min(ed);
                goali = [pid; pid];
            else
                goali = pid;
            end
        end
        f = 0;
        if pid+1 > length(grph)
            a = vtx - g(1:2);
            ed = sqrt(a(1,:).^2 + a(2, :).^2);
            [~, gid] = min(ed);
            gID = gid;
            f = 1;
        else
            if isempty(grph{pid+1})
                a = vtx - g(1:2);
                ed = sqrt(a(1,:).^2 + a(2, :).^2);
                [~, gid] = min(ed);
                gID = gid;
                f = 1;
            else
                gID = pid+1;
            end
        end
        vtx = [vtx, vtx1];
        grphi = grph{pid};
        [dmin, goalId] = getD(grph, grphi, vtx, pid, goali, 0, inf, pid, gID);
        if f == 0
            id = vtx(:, goalId(1:max(1, end-1)));
        else
            id = vtx(:, goalId);
        end
        id = [[id; ones(1, length(id(1,:)))], goalID];

        goals = [goalx(id(1, :)); goaly(id(2, :)); goalz(id(3, :))];
end
end

function [dmin, goalId] = getD(grph, grphi, vtx, goalId, goali, d, dmin, last, gID)
%     if goali(end) == length(vtx(1, :))
%         if d < dmin
%             dmin = d;
%             goalId = goali;
%         end
%     else
        i = 1;
        gr = grphi;
        if isempty(gr)
            l = 0;
        else
            l = length(gr(:, 1));
        end
        la = last;
        dd = d;
        gi = goali;
        while i <= l
            id = gr(i, :);
            if ~ismember(id(1), la)
                last = [la; id(1)];
                d = dd + id(2);
                goali = [gi; id(1)];
                if id(1) == gID
                    if d < dmin
                        dmin = d;
                        goalId = goali;
                    end
                else
                    grphi = grph{id(1)};
                    [dmin, goalId] = getD(grph, grphi, vtx, goalId, goali, d, dmin, last, gID);
                end
            end
            i = i +1;
        end
%     end
end