function [q, angle] = madgwickFilter9Axis(omega_m, acc_m, mag_m, dt, gme, gmd, q_0)
    if ~exist('q0', 'var'), q_0 = [1, 0, 0, 0]; end;  % 初期クォータニオン
    if ~exist('gme', 'var'), gme = deg2rad(5); end;   % ジャイロセンサの平均誤差 (原文 5deg/s)
    if ~exist('gmd', 'var'), gmd = deg2rad(0.2); end; % ジャイロセンサのバイアスドリフト(原文 0.2deg/(s^2))

    % フィルタ定数
    beta = sqrt(3/4) * gme; % フィルターゲイン
    zeta = sqrt(3/4) * gmd; % フィルターゲイン
    n = size(omega_m, 1);   % ステップ数
    
    % 初期値
    qSE = zeros(n, 4);  % 推定値
    qSE(1, :) = q_0;    % 初期値
     
    omg_b = [0 0 0]; % ジャイロセンサのバイアス初期値
    omg = omega_m;

    % フィルタスステップ %
    for k=2:n

        % ノルム %
        norm_acc = norm(acc_m(k, :));
        norm_mag = norm(mag_m(k, :));
    
        % 最急降下 %
        if (norm_acc > 1e-5) && (norm_mag > 1e-5) % 0付近はError
            n_acc = acc_m(k, :)/norm_acc; % 正規化加速度ベクトル
            n_mag = mag_m(k, :)/norm_mag; % 正規化地磁気べクトル
            q = qSE(k-1, :);
            
            h = qCross(qCross(q, [0,n_mag]), qConj(q));
            bx = (h(2)^2+h(3)^2)^0.5; % 水平分力
            bz = h(4); % 垂直分力
            
            % 目的関数
            fg = [2*(q(2)*q(4) - q(1)*q(3)) - n_acc(1)
                  2*(q(1)*q(2) + q(3)*q(4)) - n_acc(2)
                  2*(0.5 - q(2)^2 - q(3)^2) - n_acc(3)];
            fb = [2*bx*(0.5 - q(3)^2 - q(4)^2) + 2*bz*(q(2)*q(4) - q(1)*q(3)) - n_mag(1)
                  2*bx*(q(2)*q(3) - q(1)*q(4)) + 2*bz*(q(1)*q(2) + q(3)*q(4)) - n_mag(2)
                  2*bx*(q(1)*q(3) + q(2)*q(4)) + 2*bz*(0.5 - q(2)^2 - q(3)^2) - n_mag(3)];
            
            % ヤコビ行列
            Jg = [-2*q(3),  2*q(4), -2*q(1), 2*q(2)
                   2*q(2),  2*q(1),  2*q(4), 2*q(3)
                        0, -4*q(2), -4*q(3),	  0];
            Jb = [-2*bz*q(3)          , 2*bz*q(4)          , -4*bx*q(3)-2*bz*q(1), -4*bz*q(4)+2*bz*q(2)
                  -2*bx*q(4)+2*bz*q(2), 2*bx*q(3)+2*bz*q(1), 2*bx*q(2)+2*bz*q(4) , -2*bx*q(1)+2*bz*q(3)
                  2*bx*q(3)           , 2*bx*q(4)-4*bz*q(2), 2*bx*q(1)-4*bz*q(3) ,            2*bx*q(2)];
            
            % 結合
            fgb = [fg; fb]; % 目的関数
            Jgb = [Jg; Jb]; % ヤコビ行列
         
            nabla_f = (Jgb'*fgb); % 勾配
            n_nabla_f = nabla_f/norm(nabla_f); % 正規化
            d_qSE_ep = n_nabla_f';
            
            d_omega_b = qV(2*qCross(qConj(q), d_qSE_ep)); 
            omg_b = omg_b + zeta*d_omega_b*dt; % バイアス計算
            omg(k, :) = omg(k, :) - omg_b;     % ジャイロの補正
            
            d_qSE_omg = 0.5 * qCross(q, [0, omg(k, :)]); %クォータニオンの微分
            d_qSE_est = d_qSE_omg - beta*d_qSE_ep; % 最急降下
        
            qSE(k, :) = q + (d_qSE_est*dt);    % 変化率の積分 → 推定値更新
            qSE(k, :) = qNormalize(qSE(k, :)); % 正規化
        end
    end
    
    angle = q2Euler(qSE, 'point');
end
