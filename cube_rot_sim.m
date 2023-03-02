% Euler角はZYX系
addpath('quaternion_lib');
addpath('est_filter_lib');
clear; close all; clc;

% 動作モードフラグ
fb_mode = true;            % true:フィードバック制御, false:制御しない
filter_sim_mode = false;   % true:角度推定, false:理論値のみ
video_output_mode = true;  % true:ビデオ出力, false:出力しない

% シミュレーション係数 %
fps = 100;              % fps
dt = 1/fps;             % 刻み幅[s] (fpsに合わせる)
t_w = 10;               % シミュレーション時間 [s]
step_n = t_w*fps + 1;   % 計算ステップ数
time = (0:step_n-1)*dt; % 時間軸(0 ~ t_w)

m = 10;                 % 立方体の質量 [kg]
a = 1;                  % 立方体の辺長さ [m]
J = (1/6)*m*a^2*eye(3); % 立方体の慣性モーメン [kgm^2]

omega_0 = [2*pi, pi/3, -4*pi]; % 角速度初期値 [rad/s]
omega_ref = [0, 0, 0];         % 角速度目標値 

q_0 = [1, 0, 0, 0];   % クォータニオン初期値
q_ref = [1, 0, 0, 0]; % クォータニオン目標値

x_ref = [omega_ref, q_ref]; % 目標値

tau_0 = [0, 0, 0]; % トルク初期値 [Nm]

% フィードバックゲイン
kp = 8.0; % 比例ゲイン
kd = 3.2; % 微分ゲイン

% 測定ノイズ
ave = 0;          % 平均値
dist_omg = 0.05;  % ジャイロセンサノイズの分散
dist_acc = 0.01;  % 加速度センサノイズの分散
dist_mag = 2e7;   % 地磁気センサノイズの分散

% Helper %
iif = @(varargin)varargin{length(varargin)-varargin{1}}; % inline if(めちゃくちゃ)
omega = @(x) x(:,1:3); % 角速度抽出
quat = @(x) x(:,4:7);  % クォータニオン抽出

% システムの方程式 %
err = @(x) iif(fb_mode, [omega(x_ref)-omega(x), qCross(qConj(quat(x)), quat(x_ref))], zeros(1, 7));
inp = @(x) iif(fb_mode, -kd*omega(x)-kp*qV(quat(x)), [0,0,0]);
sys = @(x) [(-inv(J)*(cross(omega(x)', J*(omega(x)')) + inp(err(x))'))', 0.5*qCross(quat(x),[0, omega(x)])];

% main %

% パラメータの初期値 %
x = zeros(step_n, 7);   % 状態変数
x(1,:) = [omega_0, q_0];
x_e = zeros(step_n, 7); % 状態変数の誤差
x_e(1,:) = err(x(1,:)); 
u = zeros(step_n, 3);   % 入力変数
u(1,:) = tau_0;
acc = zeros(step_n, 3); % 加速度
acc(1,:) = meaAcc(q_0);
mag = zeros(step_n, 3); % 地磁気
mag(1,:) = meaMag(q_0);

for i = 2:step_n
    % 方程式を解く
    x(i,:) = rungeKuttaStep(sys, x(i-1,:), dt); % Runge=Kutta更新ステップ
    x(i,4:7) = qNormalize(quat(x(i,:))); % 正規化
    x_e(i,:) = err(x(i,:)); % 状態変数誤差
    u(i,:) = inp(x_e(i,:)); % トルク

    % センシング
    acc(i,:) = meaAcc(quat(x(i,:))); % 加速度
    mag(i,:) = meaMag(quat(x(i,:))); % 地磁気
end

if(video_output_mode), plotCubeRotVideo(quat(x), a, fps, [720, 480], 'rotate_cube.mp4', 'MPEG-4'); end % ビデオ出力(結構時間かかる)


% グラフ表示 %
figure('Name','Input and Measured Data','NumberTitle','off');

subplot(4, 1, 1);
plot(time, u');
legend('X', 'Y', 'Z');
xlabel('Time [s]');
ylabel('Torque [Nm]');

subplot(4, 1, 2);
plot(time, omega(x)');
legend('X', 'Y', 'Z');
xlabel('Time [s]');
ylabel('Angular velocity [rad/s]');

subplot(4, 1, 3);
plot(time, acc');
legend('X', 'Y', 'Z');
xlabel('Time [s]');
ylabel('Acceleration [g]');

subplot(4, 1, 4);
plot(time, mag');
legend('X', 'Y', 'Z');
xlabel('Time [s]');
ylabel('Geomagnetic field [nT]');

figure('Name','Parameter Error','NumberTitle','off');

subplot(2, 1, 1);
plot(time, omega(x)');
legend('X', 'Y', 'Z');
xlabel('Time [s]');
ylabel('Angular velocity Error[rad/s]');

subplot(2, 1, 2);
plot(time, qV(quat(x_e))');
legend('X', 'Y', 'Z');
xlabel('Time [s]');
ylabel('Vector part of quaternion Error');


figure('Name','Output Data','NumberTitle','off');

subplot(1, 1, 1);
plot(time, quat(x)'); 
legend('W', 'X', 'Y', 'Z');
xlabel('Time [s]');
ylabel('Quaternion');



% 推定とかやりたい人向けの領域
if filter_sim_mode == true
    % 測定データの作成 %
    omg_noise = whiteNoiseGen(ave, dist_omg, step_n, 3);
    acc_noise = whiteNoiseGen(ave, dist_acc, step_n, 3);
    mag_noise = whiteNoiseGen(ave, dist_mag, step_n, 3);
    omg_m = omega(x) + omg_noise;   % ジャイロセンサ
    acc_m = acc + acc_noise;          % 加速度センサ
    mag_m = mag + mag_noise;          % 地磁気センサ

    % ここからセンサフュージョン %
    sens_data = [acc_m, mag_m];

    % Acc and Mag LPF
    % alpha = 0.7;
    % sens_data = lowpassFilterFo(sens_data, alpha); % LPF

    % Non filter
    [~, angle_am] = getAngle(sens_data, 'fusion');    % 加速度センサと地磁気センサのフュージョン
    [~, angle_gyr] = getAngle(omg_m, 'integral', dt); % 角速度から積分

    % Complementary Filter
    beta = 0.65;
    angle_cf = complementaryFilter(angle_gyr, angle_am, beta);

    % Madgwick Filter
    [~, angle_mf] = madgwickFilter9Axis(omg_m, acc_m, mag_m, dt); % 本当ならgme = sqrt(dist_omg), gmd = 0にすべき

    % Extend Kalman Filter
    sens_ref = [meaAcc([1, 0, 0, 0]); meaMag([1, 0, 0, 0])]'; % リファレンスベクトル
    Q = dist_omg*eye(4); % プロセス誤差共分散
    R = [dist_acc*eye(3), zeros(3,3); zeros(3,3), dist_mag*eye(3)]; % 測定誤差共分散
    [~, angle_ekf] = kalmanFilter9Axis(omg_m, acc_m, mag_m, sens_ref, Q, R, dt); % モデル的にドリフトが乗るので要改良
    
    angle_t = rad2deg(q2Euler(quat(x), 'point')); % 真値 (ワールド座標系に対する相対角度)
    angle_si = rad2deg(angle_gyr);  % 単純積分
    angle_cf = rad2deg(angle_cf);   % Complementary Filter
    angle_mf = rad2deg(angle_mf);   % Madgwick Filter
    angle_ekf = rad2deg(angle_ekf); % Extend kalman Filter(弄ってたらうまく動かなくなった)

    % グラフ表示 %
    figure('Name','Sensor Data','NumberTitle','off');
    subplot(3, 1, 1);
    plot(time, omg_m');
    legend('X', 'Y', 'Z');
    xlabel('Time [s]');
    ylabel('Angular velocity [rad/s]');

    subplot(3, 1, 2);
    plot(time, acc_m');
    legend('X', 'Y', 'Z');
    xlabel('Time [s]');
    ylabel('Acceleration [g]');

    subplot(3, 1, 3);
    plot(time, mag_m');
    legend('X', 'Y', 'Z');
    xlabel('Time [s]');
    ylabel('Geomagnetic field [nT]');
    
    figure('Name','Estimated Euler Angle','NumberTitle','off');
    subplot(5, 1, 1);
    plot(time, angle_t');
    legend('Roll', 'Pitch', 'Yaw');
    xlabel('Time [s]');
    ylabel('True euler angle [deg]')
    
    subplot(5, 1, 2);
    plot(time, angle_si');
    legend('Roll', 'Pitch', 'Yaw');
    xlabel('Time [s]');
    ylabel('Estimated euler angle(SI) [deg]')
    
    subplot(5, 1, 3);
    plot(time, angle_cf');
    legend('Roll', 'Pitch', 'Yaw');
    xlabel('Time [s]');
    ylabel('Estimated euler angle(CF) [deg]')

    subplot(5, 1, 4);
    plot(time, angle_mf');
    legend('Roll', 'Pitch', 'Yaw');
    xlabel('Time [s]');
    ylabel('Estimated euler angle(MF) [deg]')

    subplot(5, 1, 5);
    plot(time, angle_ekf');
    legend('Roll', 'Pitch', 'Yaw');
    xlabel('Time [s]');
    ylabel('Estimated euler angle(EKF) [deg]')
end


% 関数など %

% Runge=Kutta method
function x_new = rungeKuttaStep(f, x, dt) 
    k1 = f(x);
    k2 = f(x + 0.5*k1*dt);
    k3 = f(x + 0.5*k2*dt);
    k4 = f(x + k3*dt);

    x_new = x + (dt/6)*(k1 + 2*k2 + 2*k3 + k4);
end

% 機体座標系の加速度計測(リファレンスはワールド座標系)
function new_acc = meaAcc(q, g)
    if ~exist('g', 'var'), g = 1; end
    new_acc = q2Rotmat(q, 'frame')*[0, 0, g]';
end

% 機体座標系の地磁気計測(リファレンスはワールド座標系, x軸が北, delta:伏角, theta:偏角)
function new_mag = meaMag(q, B, delta, theta)
    if ~exist('B', 'var'), B = 48000; end                 % 福井と石川の全磁力はこんな感じ(国土地理院技術資料B1-No.71)
    if ~exist('delta', 'var'), delta = deg2rad(51.5); end % 福井と石川の伏角はこんな感じ(国土地理院技術資料B1-No.71)
    if ~exist('theta', 'var'), theta = deg2rad(7.4); end  % 福井と石川の偏角は大体7.2deg ~ 7.5degくらい(国土地理院)
    Z = sin(delta); % 垂直分力
    H = cos(delta); % 水平分力
    Br = B*[H*cos(theta), H*sin(theta), Z]; % 地磁気リファレンスベクトル
    new_mag = q2Rotmat(q, 'frame')*Br';
end

% iidなホワイトノイズ
function noise = whiteNoiseGen(mu, sigma, n1, n2)
    rng(0,'twister');
    R = chol(sigma*eye(n2));
    noise = repmat(mu, n1, n2) + randn(n1, n2)*R;
end

% 立方体の頂点座標計算
function [vertex, face] = cubeRotation(q, L)
    R = q2Rotmat(q,'point');

    vert_base = [0, 0, 0; L, 0, 0; 
                 0, L, 0; 0, 0, L; 
                 L, L, 0; 0, L, L;
                 L, 0, L; L, L, L] - L/2; % 頂点座表

    face = [1, 2, 5, 3; 1, 3, 6, 4;
            1, 4, 7, 2; 4, 7, 8, 6;
            2, 5, 8, 7; 3, 6, 8, 5]; % 面と頂点の対応

    vertex = vert_base*R.'; % = (R*vert_base')';
end

% 3Dアニメーション表示&保存するやつ
function video = plotCubeRotVideo(q, a, fps, v_size, v_filename, v_format, frame_n)
    if ~exist('step', 'var'), [frame_n, ~] = size(q); end
    
    % 3Dアニメーション表示&保存
    figure('Name', v_filename, 'NumberTitle','off');

    % ビデオ設定
    video = VideoWriter(v_filename, v_format);
    video.Quality = 100;
    video.FrameRate = fps;
    set(gcf, 'Position', [0,0,v_size]);
    open(video);

    % 軸設定
    xlabel('x'); ylabel('y'); zlabel('z');
    view(3);
    axis vis3d equal;
    grid on;
    xlim([-a, a]); ylim([-a, a]); zlim([-a, a]);

    % アニメーション作成
    [vert_0, face] = cubeRotation(q(1,:), a); % 初期座標と頂点対応
    h = patch('Faces', face, 'Vertices', vert_0, 'FaceVertexCData',(1:6)','FaceColor','flat'); % 初期姿勢

    for i = 1:frame_n
        vert = cubeRotation(q(i,:), a); % 頂点座標の計算
        set(h, 'Vertices', vert); % オブジェクトの更新
        drawnow;
        frame = getframe(gcf);    % 現在フレームキャプチャ
        writeVideo(video, frame); % フレーム書き込み
    end

    close(video);
end
