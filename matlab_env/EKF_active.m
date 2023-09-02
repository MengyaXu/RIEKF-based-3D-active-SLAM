%%
clc;
clear;
close all;
addpath("cmpt_units\");
addpath("env\");
addpath("planner\");
addpath("SLAM\");

%% Initialization
index = 1;
id_f = 1;
op = 2;     % 1 predetermined; 2 active 
explore = 0;    % 1 explore; 0 min trace(P)

record = 0;
if record == 1
    video = VideoWriter(['video/EKF' num2str(index) num2str(op) num2str(explore) num2str(id_f) '.avi']);
    video.FrameRate = 5;
    video.Quality = 100;
    open(video);

    video2 = VideoWriter(['video/EKFenv' num2str(index) num2str(op) num2str(explore) num2str(id_f) '.avi']);
    video2.FrameRate = 5;
    video2.Quality = 100;
    open(video2);
end

load(['dataFeature' num2str(id_f)], 'M','feature','feature_','tri1','tri_','goalx','goaly','goalz','unknown','vertex');
% N = 221;
N = 1000;
N_elipse = 201;
map = zeros(length(goalx(1, :)), length(goaly(1, :)), length(goalz(1, :)));

% Pose = [0; 0; 0; 7; 40; 4];
Pose = [0; 0; 0; 1; 1.5; 0.5];
% Pose = [0; 0; 0; -0.5; 0; 0];
range = [0.1;4];

% goalflag = zeros(length(goalx), length(goaly), length(goalz));

load sigma sigma_head sigma_odem sigma_r
load(['noise/noise'  num2str(index)], 'noise_odem', 'noise_feat')
sigma_feat2 = [sigma_r^2; sigma_r^2; sigma_r^2];

%     h = [];
save data M op range 
num_unseen = 0;
num_vis = 0;
lock = 0;
subgoal = [];
vis_feat = [];
occ = [];
free = [];
grph{1} = [];
vtx = [2;2];
%%
er2 = [];
ef2 = [];
erP2 = 0;
efP2 = 0;
t = [];
    step_vis = 0;
for i = 1:N
    i
    fig2 = figure(2);
    f = figure(1);
%     sigma_Feat{i} = [];
    vis_line = [];
    unvis_line = [];
    z = [];
    z_est = [];
    z_delta = [];
    z_new = [];
    z_new_est = [];
    if i == 1
        x = Pose;
        for j = 1:M
            y = (R(Pose(3),'y',0)*R(Pose(2),'p',0)*R(Pose(1),'r',0))' * (feature(:, j) - Pose(4:6, i));
            zj = y;
            q = sqrt(zj' * zj);

            zj = zj + noise_feat{j, i};
            z = [z; zj];
            
%                     xfj = x(4:6) + zj;
%                     x = [x; xfj];
%                     vis_feat = [vis_feat; j];
            if q <= range(2) && q >= range(1) && y(1) > 0
                vis = reachability(feature(1:3, j), Pose(4:6, i),id_f);
                if vis == 1
                    xfj = x(4:6) + zj;
                    x = [x; xfj];
                    visble(j, i) = 1;
                    vis_line = [vis_line, Pose(4:6, i), feature(:, j)];
%                     sigma_Feat{i} = [sigma_Feat{i}; sigma_feat2];
                    vis_feat = [vis_feat; j];
                    num_vis = num_vis + 1;
                else
                    visble(j, i) = 0;
                    unvis_line = [unvis_line, Pose(4:6, i), feature(:, j)];
                end
            else
                visble(j, i) = 0;
%                 unvis_line = [unvis_line, Pose(4:6, i), feature_i(:, j)];
            end
        end
%         mi = M;
        mi = length(vis_feat);
        P = zeros(6 + 3*mi, 6 + 3*mi);
        P(7:end,7:end) = diag(repmat(sigma_feat2, mi, 1));
        P = sparse(P);
        X{1} = x;
        x_pre = x;
        xr = x(4:6);
        traceP_all = trace(P);
        
        hfg = plot3(feature(1, :), feature(2, :), feature(3, :), 'p', 'color', [0 176 80]/250, 'MarkerFaceColor', [0 176 80]/250, 'MarkerSize', 15);
        hold on;
        grid on;
%         hfall = plot3(feature_all(1, :), feature_all(2, :), feature_all(3, :), '.', 'color', [150 150 150]/250, 'MarkerSize', 5);

        hrg = plot3(Pose(4, :), Pose(5, :), Pose(6, :), 'k+-', 'MarkerSize', 10);
        hold on;

        if ~isempty(vis_line)
            hvis = plot3(vis_line(1, :), vis_line(2, :), vis_line(3, :),'-g','linewidth',1.5);
        end
        if ~isempty(unvis_line)
            hunvis = plot3(unvis_line(1, :), unvis_line(2, :), unvis_line(3, :),'--y','linewidth',1.5);
        else
            hunvis = plot3(0, 0, 0, '--y','linewidth',1.5);
        end

        hr = plot3(xr(1, :), xr(2, :), xr(3, :), 'b.-', 'MarkerSize', 10);
        rexy = plot_elipse(xr(1:2), P(2:3, 2:3));
        hrexy = plot3(rexy(1, :), rexy(2, :), xr(3, i)*ones(1, N_elipse), '.b', 'MarkerSize',0.5);
        rexz = plot_elipse(xr(1:2:3), P(2:2:4, 2:2:4));
        hrexz = plot3(rexz(1, :), xr(2, i)*ones(1, N_elipse), rexz(2, :), '.b', 'MarkerSize',0.5);
        rezy = plot_elipse(xr(2:3), P(3:4, 3:4));
        hrezy = plot3(xr(1, i)*ones(1, N_elipse), rezy(1, :), rezy(2, :), '.b', 'MarkerSize',0.5);

%         hrp = plot3(x_pre(4), x_pre(5), x_pre(6), 'mx');
%         hfp = plot3(x_pre(7:3:end), x_pre(8:3:end), x_pre(9:3:end), 'mx');
        hf = plot3(x(7:3:end), x(8:3:end), x(9:3:end), 'r*', 'MarkerSize', 10);
        hold on;
        
        r=R(x(3),'y',0)*R(x(2),'p',0)*R(x(1),'r',0);
        axis_x = x(4:6) + r*[1; 0; 0]*1;
        axis_y = x(4:6) + r*[0; 1; 0]*1;
        axis_z = x(4:6) + r*[0; 0; 1]*1;
        hroll = plot3([x(4) axis_x(1)], [x(5) axis_x(2)], [x(6) axis_x(3)], 'r-');
        hpitch = plot3([x(4) axis_y(1)], [x(5) axis_y(2)], [x(6) axis_y(3)], 'm-');
        hyell = plot3([x(4) axis_z(1)], [x(5) axis_z(2)], [x(6) axis_z(3)], 'c-');

        fexy = [];
        fexz = [];
        fezy = [];
        xy = [];
        xz = [];
        yz = [];
        for jj = 1:M
            if ismember(jj, vis_feat)
                j = find(vis_feat == jj);
                fejxy = plot_elipse(feature(1:2, jj), P(4+3*j-2:4+3*j-1, 4+3*j-2:4+3*j-1));
                fexy = [fexy, fejxy];
                xy = [xy, feature(3, jj)*ones(1, N_elipse)];
                fejxz = plot_elipse(feature(1:2:3, jj), P(4+3*j-2:2:4+3*j, 4+3*j-2:2:4+3*j));
                fexz = [fexz, fejxz];
                xz = [xz, feature(2, jj)*ones(1, N_elipse)];
                fejzy = plot_elipse(feature(2:3, jj), P(4+3*j-1:4+3*j, 4+3*j-1:4+3*j));
                fezy = [fezy, fejzy];
                yz = [yz, feature(1, jj)*ones(1, N_elipse)];
            end
        end
        hfexy = plot3(fexy(1, :), fexy(2, :), xy, '.k', 'MarkerSize',0.5);
        hfexz = plot3(fexz(1, :), xz, fexz(2, :), '.k', 'MarkerSize',0.5);
        hfezy = plot3(yz, fezy(1, :), fezy(2, :), '.k', 'MarkerSize',0.5);
        
        trisurf(tri1, vertex(1, :), vertex(2, :), vertex(3, :), 'FaceColor','k','FaceAlpha',0.5);
trisurf(tri_, feature_(:, 1), feature_(:, 2), feature_(:, 3), 'FaceColor','k','FaceAlpha',0.5);
        set(gca,'FontSize',20);
        axis([-1 13 -1 40 -0.5 1.5]);
        set(gcf,'unit','normalized','position',[0, 0, 0.65, 1], 'color', 'w');
        save data visble -append
    else
%         Q = sparse(blkdiag(sigma_head^2 * eye(3), sigma_odem^2 * eye(3)));
        Q = sparse(diag([0;0;sigma_head^2; sigma_odem^2;sigma_odem^2;0]));
        
        % prediction
        save data Q P x vis_feat i visble -append

        tic;
        if lock == 0 %&& i ~= 2
            if i == 2%i == 10
                d = inf;
                dz = inf;
            else
                d = norm(x(4:5)-goal(1:2));
                dz = abs(x(6) - goal(3));
            end
            if d < 0.5 || i == 2%i == 10
                n = 2;
%                 if i == 10
%                     lock = 0;
%                 else
%                     lock = 1;
%                 end
                [grph, vtx, unknown, occ, free, map] = newMap(grph, vtx, range, Pose(:, i-1), unknown, occ, free, map,id_f);
                    [goal, goalID] = getFront(unknown, occ, free, map,id_f);
                if isempty(goal)
                    break;
                end
    
                goals = getpath2(goalID, x(4:6), grph, vtx, map, free,id_f);
                figure(2)
                if i == 2%i == 10
                    h_un = plot3(goalx(unknown(1, :)), goaly(unknown(2, :)), goalz(unknown(3, :)), 'y.');
                    hold on;
                    h_occ = plot3(goalx(occ(1, :)), goaly(occ(2, :)), goalz(occ(3, :)), 'r.');
                    h_free = plot3(goalx(free(1, :)), goaly(free(2, :)), goalz(free(3, :)), 'b.');
                    h_v = plot3(goalx(vtx(1, :)), goaly(vtx(2, :)), goalz(ones(1,length(vtx(1, :)))), 'b*');
                    
                    %% 
                    figure(1)
                    h_sub = plot3(goals(1,2:end), goals(2,2:end), goals(3,2:end), 'o', 'color', 'm', 'MarkerFaceColor', 'm', 'MarkerSize', 5);
                else
                    set(h_un, 'XData', goalx(unknown(1, :)), 'YData',goaly(unknown(2, :)), 'ZData',goalz(unknown(3, :)))
                    set(h_occ, 'XData', goalx(occ(1, :)), 'YData',goaly(occ(2, :)), 'ZData',goalz(occ(3, :)))
                    set(h_free, 'XData', goalx(free(1, :)), 'YData',goaly(free(2, :)), 'ZData',goalz(free(3, :)))
                    set(h_v, 'XData', goalx(vtx(1, 2:end)), 'YData',goaly(vtx(2, 2:end)), 'ZData',goalz(ones(1,length(vtx(1, 2:end)))));
                    figure(1)
                    set(h_sub, 'XData', goals(1,2:end), 'YData',goals(2,2:end), 'ZData',goals(3,2:end));
                end
                plot3(goal(1), goal(2), goal(3), 'o', 'color', 'y', 'MarkerFaceColor', 'y', 'MarkerSize', 10);
            end
    
            if i == 2%i == 10
                d1 = inf;
                d1z = inf;
            else
                d1 = norm(x(4:5)-subgoal(1:2));
                d1z = abs(x(6) - subgoal(3));
            end
            if d1 < 0.5 || i == 2%i == 10 %|| sum(u(4:6, end) == 0)==3
                subgoal = goals(:, n);
    %             plot3(subgoal(1), subgoal(2), subgoal(3), 'o', 'color', 'm', 'MarkerFaceColor', 'm', 'MarkerSize', 5);
                n = n + 1;
            end
        else
%             lock = mod(lock+1, 8);
        end

        u(:, i - 1) = getV2(1, op, x, i, subgoal, explore, map, lock,id_f);
        toc;
        tc = toc;
        t = [t; tc];
        [~, Pose(:, i)] = predict(1, u(:,i-1), Q, P, Pose(:, i-1), zeros(6,1));
        vis = reachability(Pose(4:6,i-1), Pose(4:6, i),id_f);
        if vis == 0
%             u(3, i-1) = u(3, i-1) + pi/2;
            u(4:6, i-1) = [0;0;0];
            [~, Pose(:, i)] = predict(1, u(:,i-1), Q, P, Pose(:, i-1), zeros(6,1));
        end

        [P_pre, x_pre] = predict(2, u(:,i-1), Q, P, x, noise_odem(:, i));
        %feature measurement
        k = 0;
        H_feat = {};
        vis_feat_new = [];
        for j = 1:M
            y = (R(Pose(3, i),'y',0)*R(Pose(2, i),'p',0)*R(Pose(1, i),'r',0))' * (feature(1:3, j) - Pose(4:6, i));
            zj = y;
            q = sqrt(zj' * zj);
            if q <= range(2) && q >= range(1) && y(1) > 0
                vis = reachability(feature(1:3, j), Pose(4:6, i),id_f);
                if vis == 1
                    zj = zj + noise_feat{j, i};

                    visble(j, i) = 1;
                    vis_line = [vis_line, Pose(4:6, i), feature(:, j)];

                    rot = R(x_pre(3),'y',0)*R(x_pre(2),'p',0)*R(x_pre(1),'r',0);

                    if ~ismember(j, vis_feat)
%                         z_new = [z_new; zj];
                        z_new_est = [z_new_est; zj];
                        vis_feat_new = [vis_feat_new; j];
                        if length(vis_feat) == M
                            step_vis = i;
                        end
                        num_vis = num_vis + 1;
                    else
                        id = find(vis_feat == j);
                        delta = x_pre(6+3*id-2:6+3*id) - x_pre(4:6);
                        y = rot' * delta;
                        z = [z; zj];
                        z_est = [z_est; y];
                        z_delta = [z_delta; delta];

                        Rr = R(x_pre(3),'y',0)*R(x_pre(2),'p',0)*R(x_pre(1),'r',1);
                        Rp = R(x_pre(3),'y',0)*R(x_pre(2),'p',1)*R(x_pre(1),'r',0);
                        Ry = R(x_pre(3),'y',1)*R(x_pre(2),'p',0)*R(x_pre(1),'r',0);
                        Hij = [Rr'*delta Rp'*delta Ry'*delta, -rot', rot'];
                        H_feat{i, j} = Hij;
                        k = k + 1;
                    end
                else
                    visble(j, i) = 0;
                    unvis_line = [unvis_line, Pose(4:6, i), feature(:, j)];
                    num_unseen = num_unseen + 1;
                end
            else
                visble(j, i) = 0;
%                 unvis_line = [unvis_line, Pose(4:6, i), feature_i(:, j)];
                num_unseen = num_unseen + 1;
            end
        end
%         N_ = sparse(diag(z_est.^2) * diag(repmat(sigma_feat2, k, 1)));
        N_ = sparse(diag(z.^2) * sigma_r^2);
%         N_ = sparse(diag(repmat(sigma_feat2, k, 1)));
        [K, H] = KF_gain(P_pre, N_, visble(:, i), M, H_feat, i, vis_feat);
        [x, P] = KF_update(x_pre, K, z, H, P_pre, z_est, z_new_est, sigma_feat2);
%         x = x_pre;
%         P = P_pre;
X{i} = x;
%         X = [X, x];
        xr = [xr, x(4:6)];
        traceP_all = [traceP_all trace(P)];

        vis_feat = [vis_feat; vis_feat_new];
        
        
        figure(1)
            set(hfg, 'XData', feature(1, :), 'YData', feature(2, :), 'ZData', feature(3, :));
            
            
            set(hrg, 'XData', Pose(4, :), 'YData', Pose(5, :), 'ZData', Pose(6, :));

            if ~isempty(vis_line)
                set(hvis, 'XData', vis_line(1, :), 'YData', vis_line(2, :), 'ZData', vis_line(3, :));
            else
                set(hvis, 'XData', 0, 'YData', 0, 'ZData', 0);            
            end
            if ~isempty(unvis_line)
                set(hunvis, 'XData', unvis_line(1, :), 'YData', unvis_line(2, :), 'ZData', unvis_line(3, :));
            else
                set(hunvis, 'XData', 0, 'YData', 0, 'ZData', 0);     
            end

            set(hr, 'XData', xr(1, :), 'YData', xr(2, :), 'ZData', xr(3, :));
            rexy = plot_elipse(xr(1:2, i), P(4:5, 4:5));
            set(hrexy, 'XData', rexy(1, :), 'YData', rexy(2, :), 'ZData', xr(3, i)*ones(1, N_elipse));
            rezy = plot_elipse(xr(1:2:3, i), P(4:2:6, 4:2:6));
            set(hrezy, 'XData', rezy(1, :), 'YData', xr(2, i)*ones(1, N_elipse), 'ZData', rezy(2, :));
            rexz = plot_elipse(xr(2:3, i), P(5:6, 5:6));
            set(hrexz, 'XData', xr(1, i)*ones(1, N_elipse), 'YData', rexz(1, :), 'ZData', rexz(2, :));            

            r=R(x(3),'y',0)*R(x(2),'p',0)*R(x(1),'r',0);
            axis_x = x(4:6) + r*[1; 0; 0]*1;
            axis_y = x(4:6) + r*[0; 1; 0]*1;
            axis_z = x(4:6) + r*[0; 0; 1]*1;
%         hroll = plot3([x(4) axis_x(1)], [x(5) axis_x(2)], [x(6) axis_x(3)], 'r-');
%         hpitch = plot3([x(4) axis_y(1)], [x(5) axis_y(2)], [x(6) axis_y(3)], 'm-');
%         hyell = plot3([x(4) axis_z(1)], [x(5) axis_z(2)], [x(6) axis_z(3)], 'c-');
            set(hroll, 'XData', [x(4) axis_x(1)], 'YData', [x(5) axis_x(2)], 'ZData', [x(6) axis_x(3)]);
            set(hpitch, 'XData', [x(4) axis_y(1)], 'YData', [x(5) axis_y(2)], 'ZData', [x(6) axis_y(3)]);
            set(hyell, 'XData', [x(4) axis_z(1)], 'YData', [x(5) axis_z(2)], 'ZData', [x(6) axis_z(3)]);
        
%             set(hrp, 'XData', x_pre(4), 'YData', x_pre(5), 'ZData', x_pre(6));
%             set(hfp, 'XData', x_pre(7:3:end), 'YData', x_pre(8:3:end), 'ZData', x_pre(9:3:end));
            set(hf, 'XData', x(7:3:end), 'YData', x(8:3:end), 'ZData', x(9:3:end));
            fexy = [];
            fexz = [];
            fezy = [];
            xy = [];
            xz = [];
            yz = [];            
            for jj = 1:M
                if ismember(jj, vis_feat)
                    j = find(vis_feat == jj);
                    fejxy = plot_elipse(feature(1:2, jj), P(4+3*j-2:4+3*j-1, 4+3*j-2:4+3*j-1));
                    fexy = [fexy, fejxy];
                    xy = [xy, feature(3, jj)*ones(1, N_elipse)];
                    fejxz = plot_elipse(feature(1:2:3, jj), P(4+3*j-2:2:4+3*j, 4+3*j-2:2:4+3*j));
                    fexz = [fexz, fejxz];
                    xz = [xz, feature(2, jj)*ones(1, N_elipse)];
                    fejzy = plot_elipse(feature(2:3, jj), P(4+3*j-1:4+3*j, 4+3*j-1:4+3*j));
                    fezy = [fezy, fejzy];
                    yz = [yz, feature(1, jj)*ones(1, N_elipse)];
                end
            end
            set(hfexy, 'XData', fexy(1, :), 'YData', fexy(2, :), 'ZData', xy);
            set(hfexz, 'XData', fexz(1, :), 'YData', xz, 'ZData', fexz(2, :));
            set(hfezy, 'XData', yz, 'YData', fezy(1, :), 'ZData', fezy(2, :));
    end
    %     axis([0 5 0 15.5]);
    %     pause(0.5);
    if record == 1
        F(i) = getframe(f);
        writeVideo(video, F(i));
        F2(i) = getframe(fig2);
        writeVideo(video2, F2(i));
    end
    er = x(1:6) - Pose(:, i);
    er(1) = wrapToPi(er(1));
    er(2) = wrapToPi(er(2));
    er(3) = wrapToPi(er(3));
    er2 = [er2; sqrt(er' * er)];
    if i ~= 1
        erP2 = erP2 + er' * P(1:6, 1:6)^(-1) * er;
    end
end
        for jj=1:1:M
            if ismember(jj, vis_feat)
                j = find(vis_feat == jj);
                ef = x(6+3*j-2:6+3*j) - feature(:, jj);
                ef2 = [ef2; sqrt(ef' * ef)];
                efP2 = efP2 + ef' * P(6+3*j-2:6+3*j, 6+3*j-2:6+3*j)^(-1) * ef;
            end
        end
traceP = trace(P)
er_max = max(er2);
er_ave = sum(er2)/length(er2)
ef_max = max(ef2);
ef_ave = sum(ef2) / length(ef2)
    save(['result/' num2str(index) 'EKF' num2str(op) num2str(explore) num2str(id_f) '.mat'], 'u', 'traceP', 'num_unseen',...
        'er2', 'ef2', 'erP2', 'efP2', 'traceP_all', 't', 'step_vis', 'num_vis', ...
        'er_max', 'er_ave', 'ef_max', 'ef_ave', 'X');
    savefig(['fig/' num2str(index) 'EKF' num2str(op) num2str(explore) num2str(id_f) '.fig']);

                    figure(2)
                    set(h_un, 'XData',goalx(unknown(1, :)), 'YData', goaly(unknown(2, :)), 'ZData',goalz(unknown(3, :)));
                    if isempty(occ)
                        set(h_occ, 'XData',0, 'YData', 0);
                    else
                        set(h_occ, 'XData',goalx(occ(1, :)), 'YData', goaly(occ(2, :)), 'ZData',goalz(occ(3, :)));
                    end
                    set(h_free, 'XData',goalx(free(1, :)), 'YData', goaly(free(2, :)), 'ZData',goalz(free(3, :)));
    savefig(['fig/' num2str(index) 'EKF' num2str(op) num2str(explore) num2str(id_f) '_env.fig']);

if record == 1
    close(video);
    close(video2);
end
