%%
clc;
clear;
close all;

%% Initialization
index = 5;
op = 1;     % 1 predetermined; 2 active 

record = 0;
if record == 1
    video = VideoWriter(['video/' num2str(op) '.avi']);
    video.FrameRate = 5;
    video.Quality = 100;
    open(video);
end

load dataFeature2 M feature translation deform feature_all tri1
N = 221;
N_elipse = 201;

Pose = [0; 0; 0; 2; 2; 4];
% Pose = [0; 0; 0; -0.5; 0; 0];
range = [0.1;10];

load sigma sigma_head sigma_odem sigma_r
load(['noise/noise' num2str(index)], 'noise_odem', 'noise_feat')
sigma_feat2 = [sigma_r^2; sigma_r^2; sigma_r^2];
            
save data M op range
num_unseen = 0;
num_vis = 0;
vis_feat = [];
%%
er2 = [];
ef2 = [];
erP2 = 0;
efP2 = 0;
t = [];
    step_vis = 0;
for i = 1:N
    i
    f = figure(1);
    
%     sigma_Feat{i} = [];
    vis_line = [];
    unvis_line = [];
    z = [];
    z_est = [];
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
            
            if q <= range(2) && q >= range(1) && y(1) > 0
                vis = visibility(feature, Pose(4:6, i), j);
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
        mi = length(vis_feat);
        P = zeros(6 + 3*mi, 6 + 3*mi);
        P(7:end,7:end) = diag(repmat(sigma_feat2, mi, 1));
        P = sparse(P);
        X = x;
        x_pre = x;
        xr = X(4:6);
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

        hrp = plot3(x_pre(4), x_pre(5), x_pre(6), 'mx');
        hfp = plot3(x_pre(7:3:end), x_pre(8:3:end), x_pre(9:3:end), 'mx');
        hf = plot3(x(7:3:end), x(8:3:end), x(9:3:end), 'r*', 'MarkerSize', 10);
        hold on;
        
        r=R(x(3),'y',0)*R(x(2),'p',0)*R(x(1),'r',0);
        axis_x = x(4:6) + r'*[1; 0; 0]*1;
        axis_y = x(4:6) + r'*[0; 1; 0]*1;
        axis_z = x(4:6) + r'*[0; 0; 1]*1;
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


trisurf(tri1, feature(1, :), feature(2, :), feature(3, :));
        
        set(gca,'FontSize',20);
%         axis([-5 25 -15 15 -5 25]);
        set(gcf,'unit','normalized','position',[0, 0, 0.65, 1], 'color', 'w');
        save data visble -append
    else
        Q = sparse(blkdiag(sigma_head^2 * eye(3), sigma_odem^2 * eye(3)));
        
        % prediction
        save data Q P x vis_feat i visble -append

        tic;
        u(:, i - 1) = getV(1, op, x, i);
        toc;
        tc = toc;
        t = [t; tc];
        [~, Pose(:, i)] = predict(1, u(:,i-1), Q, P, Pose(:, i-1), noise_odem(:, i));
        [P_pre, x_pre] = predict(2, u(:,i-1), Q, P, x, zeros(6,1));
        %feature measurement
        k = 0;
        vis_feat_new = [];
        for j = 1:M
            y = (R(Pose(3, i),'y',0)*R(Pose(2, i),'p',0)*R(Pose(1, i),'r',0))' * (feature(1:3, j) - Pose(4:6, i));
            zj = y;
            q = sqrt(zj' * zj);
            if q <= range(2) && q >= range(1) && y(1) > 0
                vis = visibility(feature, Pose(4:6, i), j);
                if vis == 1
                    zj = zj + noise_feat{j, i};

                    visble(j, i) = 1;
                    vis_line = [vis_line, Pose(4:6, i), feature(:, j)];

                    rot = R(x_pre(3),'y',0)*R(x_pre(2),'p',0)*R(x_pre(1),'r',0);

                    if ~ismember(j, vis_feat)
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
        N_ = sparse(diag(repmat(sigma_feat2, k, 1)));
        [K, H] = KF_gain(P_pre, N_, visble(:, i), M, H_feat, i, vis_feat);
        
        % update
        [x, P] = KF_update(x_pre, K, z, H, P_pre, z_est, z_new_est, sigma_feat2);
        
        xr = [xr, x(4:6)];
        traceP_all = traceP_all + trace(P);
        
        vis_feat = [vis_feat; vis_feat_new];


        % plot
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
        hroll = plot3([x(4) axis_x(1)], [x(5) axis_x(2)], [x(6) axis_x(3)], 'r-');
        hpitch = plot3([x(4) axis_y(1)], [x(5) axis_y(2)], [x(6) axis_y(3)], 'm-');
        hyell = plot3([x(4) axis_z(1)], [x(5) axis_z(2)], [x(6) axis_z(3)], 'c-');
%             set(hroll, 'XData', [x(4) axis_x(1)], 'YData', [x(5) axis_x(2)], 'ZData', [x(6) axis_x(3)]);
%             set(hpitch, 'XData', [x(4) axis_y(1)], 'YData', [x(5) axis_y(2)], 'ZData', [x(6) axis_y(3)]);
%             set(hyell, 'XData', [x(4) axis_z(1)], 'YData', [x(5) axis_z(2)], 'ZData', [x(6) axis_z(3)]);
        
            set(hrp, 'XData', x_pre(4), 'YData', x_pre(5), 'ZData', x_pre(6));
            set(hfp, 'XData', x_pre(7:3:end), 'YData', x_pre(8:3:end), 'ZData', x_pre(9:3:end));
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
    end
    er = x(1:6) - Pose(:, i);
    er(1) = wrapToPi(er(1));
    er(2) = wrapToPi(er(2));
    er(3) = wrapToPi(er(3));
    er2 = [er2; sqrt(er' * er)];
    if i ~= 1
        erP2 = erP2 + er' * P(1:6, 1:6)^(-1) * er;
    end
    if i == N
        for jj=1:1:M
            if ismember(jj, vis_feat)
                j = find(vis_feat == jj);
                    ef = x(6+3*j-2:6+3*j) - feature(:, jj);
                    ef2 = [ef2; sqrt(ef' * ef)];
                    efP2 = efP2 + ef' * P(6+3*j-2:6+3*j, 6+3*j-2:6+3*j)^(-1) * ef;
            end
        end
    end
end
u
traceP = trace(P)
num_vis
step_vis
er_max = max(er2);
er_ave = sum(er2)/length(er2);
ef_max = max(ef2);
ef_ave = sum(ef2) / length(ef2);
if record == 1
    close(video);
    save(['result/EKF' num2str(op) '.mat'], 'u', 'traceP', 'num_unseen',...
        'er2', 'ef2', 'erP2', 'efP2', 'traceP_all', 't', 'step_vis', 'num_vis', ...
        'er_max', 'er_ave', 'ef_max', 'ef_ave');
    savefig(['fig/EKF' num2str(op) '.fig']);
end

    save(['result/' num2str(index) 'EKF' num2str(op) '.mat'], 'u', 'traceP', 'num_unseen',...
        'er2', 'ef2', 'erP2', 'efP2', 'traceP_all', 't', 'step_vis', 'num_vis', ...
        'er_max', 'er_ave', 'ef_max', 'ef_ave');
    savefig(['fig/' num2str(index) 'EKF' num2str(op) '.fig']);