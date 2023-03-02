% バックアップを取ってなかったせいで挙動が怪しい状態で保存されてしまったので注意
function [q_est, angle_est] = kalmanFilter9Axis(omg, acc, mag, ref, Q, R, dt, step)
    if ~exist('step', 'var'), step = size(omg, 1); end

    x_hat = zeros(step, 4);
    x_hat(1,:) = [1, 0, 0, 0]; % 推定クォータニオンの初期値
    P = Q; % 共分散初期値
    
    % フィルタステップ
    for k = 2:step
        Somg = vCrossMat(omg(k,:));   % ハミルトン積表現行列
        a = (acc(k,:)/norm(acc(k,:)))';
        m = (mag(k,:)/norm(mag(k,:)))';
        y = [a;m]; % 観測量ベクトル

        % 状態モデル
        F = eye(4)+(dt/2)*Somg; 
        
        % 予測ステップ
        x_hat_bar = F*x_hat(k-1,:)'; % 状態推定値
        x_hat_bar = qNormalize(x_hat_bar')'; % 正規化
        P_bar = F*P*F' ; % 推定誤差共分散

        % 観測モデル
        q = x_hat_bar';
        C = q2Rotmat(q, 'frame');
        h = [C, zeros(3,3); zeros(3,3), C];
        H = [jacobiMat(q, ref(:,1:3));
             jacobiMat(q, ref(:,4:6))]; % 線形化観測モデル

        % 更新ステップ
        e = y - h*ref'; % 観測誤差
        S = H*P_bar*H' + R;  % 観測誤差共分散
        K = (P_bar*H')/S; % カルマンゲイン
        x_hat(k,:) = (x_hat_bar+K*e)'; % 状態推定値
        x_hat(k,:) = qNormalize(x_hat(k,:)); % 正規化
        P = (eye(4) - K*H)*P_bar; % 推定誤差共分散
    end
   
    q_est = x_hat(:,:);
    angle_est = q2Euler(q_est, 'point');
end

%{
function [q_est, angle_est] = kalmanFilter9Axis(omg, acc, mag, ref, Q, R, dt, step)
    if ~exist('step', 'var'), step = size(omg, 1); end
    sigma_o = Q
    sigma_a = R(1:3,1:3);
    sigma_b = R(4:6,4:6);
    z33 = zeros(3,3); z34 = zeros(3,4);
    z43 = zeros(4,3); I3 = eye(3);

    e_q = @(q) q(2:4);
    e_q_cross = @(q) diag(e_q(q));
    sigma_k = @(q) [e_q_cross(q) + q(1)*I3; -e_q(q)];
    Qk = @(q) [(dt/2)^2*sigma_k(q)*sigma_o*sigma_k(q)',z43,z43; z34,dt*sigma_a,z33; z34,z33,dt*sigma_b]; 

    x_hat = zeros(step, 10); % [q, b_a, b_h]
    x_hat(1,1:4) = [1, 0, 0, 0]; % 推定クォータニオンの初期値

    
    P = Q; % 共分散初期値
    
    % フィルタステップ
    for k = 2:step
        q = x_hat(k-1,1:4); % クォータニオンの取り出し
        Somg = vCrossMat(omg(k,:));   % ハミルトン積表現行列
        a = (acc(k,:)/norm(acc(k,:)))';
        m = (mag(k,:)/norm(mag(k,:)))';
        y = [a;m]; % 観測量ベクトル

        % 状態モデル
        f = eye(4)+(dt/2)*Somg;

        F = [  f, z43, z43;
             z34,  I3, z33;
             z34, z33,  I3];
        
        
        % 観測モデル
        z63 = zeros(6,3);
        h = [jacobiMat(q, ref(:,1:3));
             jacobiMat(q, ref(:,4:6))];
        H = [h, z63, z63]; % 観測モデル

        % 予測ステップ
        x_hat_bar = F*x_hat(k-1,:)'; % 状態推定値
        x_hat_bar(1:4) = qNormalize(x_hat_bar(1:4)')'; % 正規化
        P_bar = F*P*F' + Qk(x_hat_bar(1:4)'); % 推定誤差共分散

        % 更新ステップ
        e = y - H*x_hat_bar; % 観測誤差
        S = H*P_bar*H' + R;  % 観測誤差共分散
        K = P_bar*H'/S; % カルマンゲイン
        x_hat(k,:) = (x_hat_bar+K*e)'; % 状態推定値
        x_hat(k,1:4) = qNormalize(x_hat(k,1:4)); % 正規化
        P = (eye(10) - K*H)*P_bar; % 推定誤差共分散
    end
   
    q_est = x_hat(:,1:4);
    angle_est = q2Euler(q_est, 'point');
end
%}