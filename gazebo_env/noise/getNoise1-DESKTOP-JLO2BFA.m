sigma_head = 0.01;
sigma_odem = 0.05;
sigma_r = 0.02;

save sigma sigma_head sigma_odem sigma_r %-append
noise_odem = [];
% noise_smoo = [];
% noise_cons = [];
% noise_feat = [];
for i = 1:5000
    noise_odem = [noise_odem, [normrnd(0, sigma_head, 3, 1);normrnd(0, sigma_odem, 3, 1)]];
    while noise_odem(1, i) > 2*sigma_head || noise_odem(1, i) < -2*sigma_head
        noise_odem(1, i) = normrnd(0, sigma_head, 1, 1);
    end
    while noise_odem(2, i) > 2*sigma_head || noise_odem(2, i) < -2*sigma_head
        noise_odem(2, i) = normrnd(0, sigma_head, 1, 1);
    end
    while noise_odem(3, i) > 2*sigma_head || noise_odem(3, i) < -2*sigma_head
        noise_odem(3, i) = normrnd(0, sigma_head, 1, 1);
    end
    while noise_odem(4, i) > 2*sigma_odem || noise_odem(4, i) < -2*sigma_odem
        noise_odem(4, i) = normrnd(0, sigma_odem, 1, 1);
    end
    while noise_odem(5, i) > 2*sigma_odem || noise_odem(5, i) < -2*sigma_odem
        noise_odem(5, i) = normrnd(0, sigma_odem, 1, 1);
    end
    while noise_odem(6, i) > 2*sigma_odem || noise_odem(6, i) < -2*sigma_odem
        noise_odem(6, i) = normrnd(0, sigma_odem, 1, 1);
    end
    
    for j = 1:200
        noise_feat{j, i} = normrnd(0, sigma_r, 3, 1);
        while noise_feat{j, i}(1) > 2*sigma_r || noise_feat{j, i}(1) < -2*sigma_r
            noise_feat{j, i}(1) = normrnd(0, sigma_r, 1, 1);
        end
        while noise_feat{j, i}(2) > 2*sigma_r || noise_feat{j, i}(2) < -2*sigma_r
            noise_feat{j, i}(2) = normrnd(0, sigma_r, 1, 1);
        end 
        while noise_feat{j, i}(3) > 2*sigma_r || noise_feat{j, i}(3) < -2*sigma_r
            noise_feat{j, i}(3) = normrnd(0, sigma_r, 1, 1);
        end 
    end
end
save noise/noise2 noise_odem  noise_feat %-append