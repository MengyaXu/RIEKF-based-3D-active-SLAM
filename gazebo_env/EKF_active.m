%%
clc;
clear;
close all;
addpath("cmpt_units\");
addpath("env\");
addpath("planner\");
addpath("SLAM\");

%% Initialization
index = 2;
id_f = 1;
op = 2;     % 1 predetermined; 2 active 
explore = 0;    % 1 explore; 0 min trace(P)

record = 1;
if record == 1
    video = VideoWriter(['video/EKF' num2str(index) num2str(op) num2str(explore) '.avi']);
    video.FrameRate = 5;
    video.Quality = 100;
    open(video);

    video2 = VideoWriter(['video/EKFenv' num2str(index) num2str(op) num2str(explore) '.avi']);
    video2.FrameRate = 5;
    video2.Quality = 100;
    open(video2);
end

M = 10;
N = 100;
N_elipse = 201;

visble = zeros(M,N);

% setenv('ROS_MASTER_URI','http://192.168.125.128:11311')
% setenv('ROS_IP','192.168.125.1')
% rosinit
odomScrib = rossubscriber('/odom');
odomdata = receive(odomScrib,10);
odom0 = getPose(odomdata.Pose);
Odom = odom0;

poseScrib = rossubscriber('/gazebo/model_states');
posedata = receive(poseScrib,10);
pose0 = getPose(posedata);
Pose = pose0;

pd = 0.5;
feature = getFeature(posedata, id_f, pd);
load(['dataFeature' num2str(id_f)], 'goalx','goaly','goalz', 'unknown');
map = -1 * ones(length(goalx(1, :)), length(goaly(1, :)), length(goalz(1, :)));

fScrib = rossubscriber('/feature');
robot = rospublisher('/mobile_base/commands/velocity');
velmsg = rosmessage(robot);

scScrib = rossubscriber('scan');
range = [0.1;4];

load sigma sigma_head sigma_odem sigma_r
load(['noise/noise' num2str(index)], 'noise_odem', 'noise_feat')
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
    vis_line = [];
    unvis_line = [];
    z = [];
    z_est = [];
    z_delta = [];
    z_new = [];
    z_new_est = [];
    if i == 1
        x = Pose;
        fdata = receive(fScrib,10);
        f = fdata.Data;
        if isempty(f)
            P = zeros(6, 6);
        else
            for j = 1:4:length(f)
                id = f(j);
                zj = f(j+1:j+3) + [0;0;0.3];
                zj = zj + noise_feat{id, i};

%             y = (R(Pose(3),'y',0)*R(Pose(2),'p',0)*R(Pose(1),'r',0))' * (feature(:, id) - Pose(4:6, i));
%             zj = y;
%             q = sqrt(zj' * zj);
%             zj = zj + noise_feat{j, i};

                z = [z; zj];

                rot = R(x(3),'y',0)*R(x(2),'p',0)*R(x(1),'r',0);
                delta = rot * zj;
                xfj = x(4:6) + delta;
                x = [x; xfj];
                visble(id, i) = 1;
                vis_line = [vis_line, Pose(4:6, i), feature(:,id)];
    %                     sigma_Feat{i} = [sigma_Feat{i}; sigma_feat2];
                vis_feat = [vis_feat; id];
                num_vis = num_vis + 1;
            end
%         mi = M;
            mi = length(vis_feat);
            P = zeros(6 + 3*mi, 6 + 3*mi);
            P(7:end,7:end) = diag(repmat(sigma_feat2, mi, 1));
        end
        P = sparse(P);
        X = x;
        x_pre = x;
        xr = X(4:6);
        traceP_all = trace(P);
        
        figure(1);
        hfg = plot3(feature(1, :), feature(2, :), feature(3, :), 'p', 'color', [0 176 80]/250, 'MarkerFaceColor', [0 176 80]/250, 'MarkerSize', 15);
        hold on;
%         hfall = plot3(feature_all(1, :), feature_all(2, :), feature_all(3, :), '.', 'color', [150 150 150]/250, 'MarkerSize', 5);

        hrg = plot3(Pose(4, :), Pose(5, :), Pose(6, :), 'k+-', 'MarkerSize', 10);
        hold on;
        grid on;

        if ~isempty(vis_line)
            hvis = plot3(vis_line(1, :), vis_line(2, :), vis_line(3, :),'-g','linewidth',1.5);
        else
            hvis = plot3(0, 0, 0, '--g','linewidth',1.5);
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

        hrp = plot3(x_pre(4), x_pre(5), x_pre(6), 'mx');
%         hfp = plot3(x_pre(7:3:end), x_pre(8:3:end), x_pre(9:3:end), 'mx');
        if isempty(f)
            hf = plot3(0, 0, 0, 'r*', 'MarkerSize', 10);
        else
            hf = plot3(x(7:3:end), x(8:3:end), x(9:3:end), 'r*', 'MarkerSize', 10);
        end
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
        if isempty(vis_feat)
            fexy = [0;0];
            fexz = [0;0];
            fezy = [0;0];
            xy = 0;
            xz = 0;
            yz = 0;
        else
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
        end
        hfexy = plot3(fexy(1, :), fexy(2, :), xy, '.k', 'MarkerSize',0.5);
        hfexz = plot3(fexz(1, :), xz, fexz(2, :), '.k', 'MarkerSize',0.5);
        hfezy = plot3(yz, fezy(1, :), fezy(2, :), '.k', 'MarkerSize',0.5);
        
        set(gca,'FontSize',20);
%         axis([-5 25 -15 15 -5 25]);
        set(gcf,'unit','normalized','position',[0, 0, 0.65, 1], 'color', 'w');
        save data visble -append
    else
        Q = sparse(diag([0;0;sigma_head^2; sigma_odem^2;sigma_odem^2;0]));
        
        scdata = receive(scScrib,2);
        scan = lidarScan(scdata);
        ranges = scan.Ranges;
        ranges(isnan(ranges)) = scdata.RangeMax;
        angles = scan.Angles(20:end-20);
                [~, ~, unknown, occ, free, map] = newMap(1, grph, vtx, map, Pose(:, i-1), unknown, occ, free, ranges, angles,id_f,range);

        % prediction
        save data Q P x vis_feat i visble -append

        t_plan = tic;
        if lock == 0 %&& i ~= 2
            if i == 2%i == 10
                d = inf;
            else
                d = norm(x(4:5)-goal(1:2));
            end
            if (d < 0.3) || i == 2%i == 10
%                 figure(1)
%                 mapHandle.CData = occupancyMatrix(map);
%                 title(axesHandle, ['OccupancyGrid: Update ' num2str(i)]);
%                 mapm = occupancyMatrix(map,'ternary'); % 0 free; 1 occ; -1 unknown

                n = 2;
                [grph, vtx, unknown, occ, free, map] = newMap(2, grph, vtx, map, Pose(:, i-1), unknown, occ, free, ranges, angles,id_f,range);
                    [goal, goalID] = getFront(unknown, occ, free, map,id_f);
                if isempty(goal)
                    break;
                end
    
                goals = getpath2(goalID, x(4:6), grph, vtx, map, free,id_f);
                figure(2)
                if i == 2%i == 10
                    h_un = plot(goalx(unknown(1, :)), goaly(unknown(2, :)), 'y.');
                    hold on;
                    if isempty(occ)
                        h_occ = plot(0, 0, 'r.');
                    else
                        h_occ = plot(goalx(occ(1, :)), goaly(occ(2, :)), 'r.');
                    end
                    h_free = plot(goalx(free(1, :)), goaly(free(2, :)), 'b.');
                    h_v = plot(goalx(vtx(1, :)), goaly(vtx(2, :)), 'b*');
                    
                    figure(1)
                    h_sub = plot(goals(1,2:end), goals(2,2:end), 'o', 'color', 'm', 'MarkerFaceColor', 'm', 'MarkerSize', 5);
                else
                    set(h_un, 'XData',goalx(unknown(1, :)), 'YData', goaly(unknown(2, :)));
                    if isempty(occ)
                        set(h_occ, 'XData',0, 'YData', 0);
                    else
                        set(h_occ, 'XData',goalx(occ(1, :)), 'YData', goaly(occ(2, :)));
                    end
                    set(h_free, 'XData',goalx(free(1, :)), 'YData', goaly(free(2, :)));
                    set(h_v, 'XData',goalx(vtx(1, 2:end)), 'YData', goaly(vtx(2, 2:end)));
                    figure(1)
                    set(h_sub, 'XData', goals(1,2:end), 'YData',goals(2,2:end));
                end
                figure(1)
                plot3(goal(1), goal(2), goal(3), 'o', 'color', 'y', 'MarkerFaceColor', 'y', 'MarkerSize', 10);
            end
    
            if i == 2%i == 10
                d1 = inf;
            else
                d1 = norm(x(4:5)-subgoal(1:2));
            end
            if (d1 < 0.3 ) || i == 2%i == 10 %|| sum(u(4:6, end) == 0)==3
                subgoal = goals(:, n);
    %             plot3(subgoal(1), subgoal(2), subgoal(3), 'o', 'color', 'm', 'MarkerFaceColor', 'm', 'MarkerSize', 5);
                n = n + 1;
            end
        else
%             lock = mod(lock+1, 8);
        end

        [u(:, i - 1),ur(:, i - 1)] = getV2(1, op, x, i, subgoal, explore, map, lock,id_f, ranges, angles);
        tc = toc(t_plan);
        t = [t; tc];

        move(ur(:,i-1), robot, velmsg);
        posedata = receive(poseScrib,10);
        Pose(:,i) = getPose(posedata);

%         odomdata = receive(odomScrib,10);
%         odom = getPose(odomdata.Pose);
%         d_odom = odom - Odom(:, end);
%         Odom = [Odom, odom];
        [P_pre, x_pre] = predict(2, u(:,i-1), Q, P, x, noise_odem(:, i));
%         [P_pre, x_pre] = RIEKF_predict(2, u(:,i-1), Q, P, x, zeros(6,1));
%         xr_pre = x(1:6) + d_odom;
%         x_pre(1:6) = xr_pre;

        %feature measurement
        k = 0;
        vis_feat_new = [];
        fdata = receive(fScrib,10);
        f = fdata.Data;
        H_feat = {};
        if ~isempty(f)
            for j = 1:4:length(f)
                id = f(j);
                zj = f(j+1:j+3) + [0;0;0.3];
                zj = zj + noise_feat{id, i};

                visble(id, i) = 1;
                vis_line = [vis_line, Pose(4:6, i), feature(:, id)];

                rot = R(x_pre(3),'y',0)*R(x_pre(2),'p',0)*R(x_pre(1),'r',0);

                if ~ismember(id, vis_feat)
                    z_new_est = [z_new_est; zj];
                    vis_feat_new = [vis_feat_new; id];
                    if length(vis_feat) + length(vis_feat_new) == M
                        step_vis = i;
                    end
                    num_vis = num_vis + 1;
                else
                    fid = find(vis_feat == id);
                    delta = x_pre(6+3*fid-2:6+3*fid) - x_pre(4:6);
                    y = rot' * delta;
                    z = [z; zj];
                    z_est = [z_est; y];
                    z_delta = [z_delta; delta];

                    Rr = R(x_pre(3),'y',0)*R(x_pre(2),'p',0)*R(x_pre(1),'r',1);
                    Rp = R(x_pre(3),'y',0)*R(x_pre(2),'p',1)*R(x_pre(1),'r',0);
                    Ry = R(x_pre(3),'y',1)*R(x_pre(2),'p',0)*R(x_pre(1),'r',0);
                    Hij = [Rr'*delta Rp'*delta Ry'*delta, -rot', rot'];
                    H_feat{i, id} = Hij;
                    k = k + 1;
                end
            end
                
        end
%         N_ = sparse(diag(z_est.^2) * diag(repmat(sigma_feat2, k, 1)));
        N_ = sparse(diag(z.^2) * sigma_r^2);
%         N_ = sparse(diag(repmat(sigma_feat2, k, 1)));
        [K, H] = KF_gain(P_pre, N_, visble(:, i), M, H_feat, i, vis_feat);
        [x, P] = KF_update(x_pre, K, z, H, P_pre, z_est, z_new_est, sigma_feat2);

        X{i} = x;
        xr = [xr, x(4:6)];
        traceP_all = [traceP_all trace(P)];

        vis_feat = [vis_feat; vis_feat_new];
        
        
        figure(1)
            set(hrg, 'XData', Pose(4, :), 'YData', Pose(5, :), 'ZData', Pose(6, :));
            if ~isempty(vis_line)
                set(hvis, 'XData', vis_line(1, :), 'YData', vis_line(2, :), 'ZData', vis_line(3, :));
            else
                set(hvis, 'XData', 0, 'YData', 0, 'ZData', 0);            
            end
%             if ~isempty(unvis_line)
%                 set(hunvis, 'XData', unvis_line(1, :), 'YData', unvis_line(2, :), 'ZData', unvis_line(3, :));
%             else
%                 set(hunvis, 'XData', 0, 'YData', 0, 'ZData', 0);     
%             end

            set(hr, 'XData', xr(1, :), 'YData', xr(2, :), 'ZData', xr(3, :));
            rexy = plot_elipse(xr(1:2, i), P(4:5, 4:5));
            set(hrexy, 'XData', rexy(1, :), 'YData', rexy(2, :), 'ZData', xr(3, i)*ones(1, N_elipse));
            rezy = plot_elipse(xr(1:2:3, i), P(4:2:6, 4:2:6));
            set(hrezy, 'XData', rezy(1, :), 'YData', xr(2, i)*ones(1, N_elipse), 'ZData', rezy(2, :));
            rexz = plot_elipse(xr(2:3, i), P(5:6, 5:6));
            set(hrexz, 'XData', xr(1, i)*ones(1, N_elipse), 'YData', rexz(1, :), 'ZData', rexz(2, :));            

            r=R(x(3),'y',0)*R(x(2),'p',0)*R(x(1),'r',0);
            axis_x = x(4:6) + r*[1; 0; 0]*0.1;
            axis_y = x(4:6) + r*[0; 1; 0]*0.1;
            axis_z = x(4:6) + r*[0; 0; 1]*0.1;
            set(hroll, 'XData', [x(4) axis_x(1)], 'YData', [x(5) axis_x(2)], 'ZData', [x(6) axis_x(3)]);
            set(hpitch, 'XData', [x(4) axis_y(1)], 'YData', [x(5) axis_y(2)], 'ZData', [x(6) axis_y(3)]);
            set(hyell, 'XData', [x(4) axis_z(1)], 'YData', [x(5) axis_z(2)], 'ZData', [x(6) axis_z(3)]);
        
            set(hrp, 'XData', x_pre(4), 'YData', x_pre(5), 'ZData', x_pre(6));
%             set(hfp, 'XData', x_pre(7:3:end), 'YData', x_pre(8:3:end), 'ZData', x_pre(9:3:end));
            set(hf, 'XData', x(7:3:end), 'YData', x(8:3:end), 'ZData', x(9:3:end));
            fexy = [];
            fexz = [];
            fezy = [];
            xy = [];
            xz = [];
            yz = [];            
            if isempty(vis_feat)
                fexy = [0;0];
                fexz = [0;0];
                fezy = [0;0];
                xy = 0;
                xz = 0;
                yz = 0;
            else
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
            end
            set(hfexy, 'XData', fexy(1, :), 'YData', fexy(2, :), 'ZData', xy);
            set(hfexz, 'XData', fexz(1, :), 'YData', xz, 'ZData', fexz(2, :));
            set(hfezy, 'XData', yz, 'YData', fezy(1, :), 'ZData', fezy(2, :));
    end
    if record == 1
        F(i) = getframe(fig);
        writeVideo(video, F(i));

        F2(i) = getframe(fig2);
        writeVideo(video2, F2(i));
    end

    er = x(1:6) - Pose(:, i);
    er(1) = wrapToPi(er(1));
    er(2) = wrapToPi(er(2));
    er(3) = wrapToPi(er(3));
    er2 = [er2; sqrt(er' * er)];
%     if i ~= 1
%         erP2 = erP2 + er' * P(1:6, 1:6)^(-1) * er;
%     end
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

save(['result/' num2str(index) 'EKF' num2str(op) num2str(explore) num2str(id_f) '.mat'], 'u', 'ur', 'traceP', 'num_unseen',...
        'er2', 'ef2', 'erP2', 'efP2', 'traceP_all', 't', 'step_vis', 'num_vis', ...
        'er_max', 'er_ave', 'ef_max', 'ef_ave', 'X');
figure(1)
savefig(['fig/' num2str(index) 'EKF' num2str(op) num2str(explore) num2str(id_f) '.fig']);

    figure(2)
    set(h_un, 'XData',goalx(unknown(1, :)), 'YData', goaly(unknown(2, :)));
    if isempty(occ)
        set(h_occ, 'XData',0, 'YData', 0);
    else
        set(h_occ, 'XData',goalx(occ(1, :)), 'YData', goaly(occ(2, :)));
    end
    set(h_free, 'XData',goalx(free(1, :)), 'YData', goaly(free(2, :)));
savefig(['fig/' num2str(index) 'EKF' num2str(op) num2str(explore) num2str(id_f) '_env.fig']);

if record == 1
    close(video);
    close(video2);
end

%     save(['result/' num2str(index) 'RIEKF' num2str(op) num2str(explore) num2str(id_f) '.mat'], 'u', 'traceP', 'num_unseen',...
%         'er2', 'ef2', 'erP2', 'efP2', 'traceP_all', 't', 'step_vis', 'num_vis', ...
%         'er_max', 'er_ave', 'ef_max', 'ef_ave');
%     savefig(['fig/' num2str(index) 'RIEKF' num2str(op) num2str(explore) num2str(id_f) '.fig']);