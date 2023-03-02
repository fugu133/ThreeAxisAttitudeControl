function [q, angle] = getAngle(sens_data, calc_mode, dt, q_0)
    if ~exist('dt', 'var'), dt = 0; end
    if ~exist('q_0', 'var'), q_0 = [1, 0, 0, 0]; end

    if strcmp(calc_mode, 'integral')
        % 角速度から角度計算
        omega_m = sens_data;
        n = size(omega_m,1);
        q_est = zeros(n, 4);
        q_est(1,:) = q_0;

        % 積分ステップ
        for i=2:n
            d_q_est = 0.5 * qCross(q_est(i-1,:), [0, omega_m(i-1, :)]); % クォータニオンの微分
            q_est(i,:) = qNormalize(q_est(i-1,:) + d_q_est*dt);         % 積分
        end
        angle = q2Euler(q_est, 'point'); % ワールド座標系に対する相対角度
        q = q_est;

    elseif strcmp(calc_mode, 'fusion')
        n = size(sens_data,1);
        
        % ノルム 2017aはvecnormが使えない
        acc_m_n = zeros(n,3);
        mag_m_n = zeros(n,3);
        for i = 1:n
            acc_m_n(i,:) = sens_data(i,1:3) / norm(sens_data(i,1:3));
            mag_m_n(i,:) = sens_data(i,4:6) / norm(sens_data(i,4:6));
        end

        % 加速度によるroll角 pitch角計算
        angle_acc = [atan2(acc_m_n(:,2), acc_m_n(:,3)), -atan2(acc_m_n(:,1), sqrt(acc_m_n(:,2).^2 + acc_m_n(:,3).^2))]; 
        
        % 地磁気によるyaw角計算
        % H = (mag_m_n(:,1).^2 + mag_m_n(:,2).^2).^0.5; % 水平分力
        % Z = mag_m_n(:,3); % 垂直分力
        % mag_m_n = [H, zeros(size(H)), Z];
        bfy = mag_m_n(:,2).*cos(angle_acc(:,1)) - mag_m_n(:,3).*sin(angle_acc(:,1));
        bfx = mag_m_n(:,1).*cos(angle_acc(:,2)) + mag_m_n(:,2).*sin(angle_acc(:,1)).*sin(angle_acc(:,2)) + mag_m_n(:,3).*sin(angle_acc(:,2)).*cos(angle_acc(:,1));
        angle_mag = atan2(-bfy, bfx);
        
        angle = [angle_acc, angle_mag]; % 結合
        q = euler2Q(angle, 'frame');
    end
end