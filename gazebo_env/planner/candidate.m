function [V_f, U_f] = candidate(V, U, Xr, goal, explore,id_f, mapm)
delta = 0.45;
% load(['dataFeature' num2str(id_f)], 'feature_','tri1','vertex','id1','tri_');

% load data i h

% face = tri1;
% 
% vertices = vertex;
% [d, v] = point2trimesh('Face', face, 'Vertices', vertices', 'QueryPoints', Xr(4:6, :)');
% [d2, v2] = point2trimesh('Face', tri_, 'Vertices', feature_, 'QueryPoints', Xr(4:6, :)');

% index = find(abs(d) <= delta);
% index2 = find(abs(d2) <= delta);
% index3 = [index; index2];

Xr_last = Xr;
% Xr(:, index) = [];
% V(:, index) = [];
Xr(:, index3) = [];
V(:, index3) = [];
U(:, index3) = [];
% % h = [];
% % if i == 2
% figure(3)
%     h = plot3(Xr_last(4, :), Xr_last(5, :), Xr_last(6, :), 'm.');
% % else
%     set(h, 'XData', Xr_last(4, :), 'YData', Xr_last(5,:), 'ZData', Xr_last(6,:));
%     set(h, 'XData', Xr(4, :), 'YData', Xr(5,:), 'ZData', Xr(6,:));
% % end
% % save data h -append

d_goal = [];
for i = 1:length(Xr(1, :))
    xr = Xr(4:6, i);
    di_goal = norm(goal - xr);
    d_goal = [d_goal; di_goal];
end
[d_g, id_g] = sort(d_goal);
l = length(Xr(1, :));
if explore == 0
    V_f = V(:, id_g(1: l/5));
    U_f = U(:, id_g(1: l/5));
    Xr_f = Xr(:, id_g(1: l/5));
else
    V_f = V(:, id_g(1));
    U_f = U(:, id_g(1));
    Xr_f = Xr(:, id_g(1));
end

