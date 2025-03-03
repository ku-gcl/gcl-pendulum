close all;
clear all;

addpath(strcat(pwd), '../');
plotsettings;

% 表示する時間幅（秒）
time_window = 15;

%% プロットの設定
fig1 = figure;  % メインのプロット用フィギュア

% Axesのハンドルを保存
ax(1) = subplot(3, 2, 1);
hold(ax(1), 'on');
xlabel(ax(1), 'Time (s)');
ylabel(ax(1), 'Attitude Angle (deg)');
theta_p_plot = plot(ax(1), NaN, NaN, 'b', 'DisplayName', "$\theta_p$");
theta_p_kf_plot = plot(ax(1), NaN, NaN, 'r', 'DisplayName', "$\theta_{p_{kf}}$");
legend(ax(1));
grid(ax(1), 'on');

ax(2) = subplot(3, 2, 2);
hold(ax(2), 'on');
xlabel(ax(2), 'Time (s)');
ylabel(ax(2), 'Angular Velocity (deg/s)');
theta_p_dot_plot = plot(ax(2), NaN, NaN, 'b', 'DisplayName', "$\dot \theta_p$");
theta_p_dot_kf_plot = plot(ax(2), NaN, NaN, 'r', 'DisplayName', "$\dot \theta_{p_{kf}}$");
legend(ax(2));
grid(ax(2), 'on');

ax(3) = subplot(3, 2, 3);
hold(ax(3), 'on');
xlabel(ax(3), 'Time (s)');
ylabel(ax(3), 'Wheel Angle (deg)');
theta_w_plot = plot(ax(3), NaN, NaN, 'b', 'DisplayName', "$\theta_w$");
theta_w_kf_plot = plot(ax(3), NaN, NaN, 'r', 'DisplayName', "$\theta_{w_{kf}}$");
legend(ax(3));
grid(ax(3), 'on');

ax(4) = subplot(3, 2, 4);
hold(ax(4), 'on');
xlabel(ax(4), 'Time (s)');
ylabel(ax(4), 'Wheel Angular Velocity (deg/s)');
theta_w_dot_plot = plot(ax(4), NaN, NaN, 'b', 'DisplayName', "$\dot \theta_w$");
theta_w_dot_kf_plot = plot(ax(4), NaN, NaN, 'r', 'DisplayName', "$\dot \theta_{w_{kf}}$");
legend(ax(4));
grid(ax(4), 'on');

ax(5) = subplot(3, 2, 5);
hold(ax(5), 'on');
xlabel(ax(5), 'Time (s)');
ylabel(ax(5), 'Motor Value / PWM Duty');
log_motor_value_plot = plot(ax(5), NaN, NaN, 'b', 'DisplayName', "Motor Value");
log_pwm_duty_plot = plot(ax(5), NaN, NaN, 'r', 'DisplayName', "PWM Duty");
legend(ax(5));
grid(ax(5), 'on');

ax(6) = subplot(3, 2, 6);
hold(ax(6), 'on');
xlabel(ax(6), 'Time (s)');
ylabel(ax(6), 'Motor Direction');
log_motor_direction_plot = plot(ax(6), NaN, NaN, 'b', 'DisplayName', "Motor Direction");
legend(ax(6));
grid(ax(6), 'on');

%% 倒立振子の描画 ----------------------------------------
fig2 = figure;  % 倒立振子用の別フィギュア
hold on;

% 直方体のサイズ
width = 15;
height_pole = 20;
depth = 0.5;

% 円柱のパラメータ
r_wheel = 2.8;

% ポールとホイールの初期位置
center = [0 r_wheel];

% ポールのポリゴン
pole_shape = polyshape([-depth/2 depth/2 depth/2 -depth/2], [0 0 height_pole height_pole]);

% ホイールの円
theta = linspace(0, 2*pi, 100);
wheel_x = r_wheel * cos(theta);
wheel_y = r_wheel * sin(theta);

% ホイールの目印ライン
line_wheel_x = [0, 0];
line_wheel_y = [0, r_wheel];

% ポール、ホイール、ラインのパッチオブジェクトを作成
h_pole = patch('XData', pole_shape.Vertices(:,1), 'YData', pole_shape.Vertices(:,2) + r_wheel, 'FaceColor', 'cyan');
h_wheel = patch('XData', wheel_x, 'YData', wheel_y, 'FaceColor', 'red');
h_line_wheel = plot(line_wheel_x, line_wheel_y, 'k', 'LineWidth', 2);

% 軸の設定
axis equal;
xlabel('X');
ylabel('Y');
xlim([-40, 40]);
ylim([-10, 40]);
grid on;

drawnow;

%% データ保存用の変数 ----------------------------------------
time_data = [];
theta_p_data = [];
theta_p_dot_data = [];
theta_w_data = [];
theta_w_dot_data = [];
theta_p_kf_data = [];
theta_p_dot_kf_data = [];
theta_w_kf_data = [];
theta_w_dot_kf_data = [];
log_motor_value_data = [];
log_pwm_duty_data = [];
log_motor_direction_data = [];

%% UDP受信ポートの設定
PORT = 12345;
u = udpport("LocalPort", PORT); % C++側で設定したポート番号
disp('Waiting for data...');

% データを受信して表示
while true
    if u.NumBytesAvailable > 0
        data = read(u, u.NumBytesAvailable, "string");
        % 受信したデータを解析して数値に変換
        data_array = split(data, "data;");
        tokens = split(data_array(end), ';');
        elapsed_time = str2double(tokens(1));
        theta_p = str2double(tokens(2));        % rad
        theta_p_dot = str2double(tokens(3));    % rad/s
        theta_w = str2double(tokens(4));        % rad
        theta_w_dot = str2double(tokens(5));    % rad/s
        theta_p_kf = str2double(tokens(6));     % rad
        theta_p_dot_kf = str2double(tokens(7)); % rad/s
        theta_w_kf = str2double(tokens(8));     % rad
        theta_w_dot_kf = str2double(tokens(9)); % rad/s
        log_motor_value = str2double(tokens(10));
        log_motor_direction = str2double(tokens(11));
        log_pwm_duty = str2double(tokens(12));
        
        % データを保存
        time_data = [time_data elapsed_time];
        theta_p_data = [theta_p_data rad2deg(theta_p)];
        theta_p_kf_data = [theta_p_kf_data rad2deg(theta_p_kf)];
        theta_p_dot_data = [theta_p_dot_data rad2deg(theta_p_dot)];
        theta_p_dot_kf_data = [theta_p_dot_kf_data rad2deg(theta_p_dot_kf)];
        theta_w_data = [theta_w_data rad2deg(theta_w)];
        theta_w_kf_data = [theta_w_kf_data rad2deg(theta_w_kf)];
        theta_w_dot_data = [theta_w_dot_data rad2deg(theta_w_dot)];
        theta_w_dot_kf_data = [theta_w_dot_kf_data rad2deg(theta_w_dot_kf)];
        log_motor_value_data = [log_motor_value_data log_motor_value];
        log_pwm_duty_data = [log_pwm_duty_data log_pwm_duty];
        log_motor_direction_data = [log_motor_direction_data log_motor_direction];
        
        % 直近time_window秒のデータに限定
        if elapsed_time > time_window
            idx = find(time_data > elapsed_time - time_window, 1); % 最初のインデックスを取得
            time_data = time_data(idx:end);
            theta_p_data = theta_p_data(idx:end);
            theta_p_kf_data = theta_p_kf_data(idx:end);
            theta_p_dot_data = theta_p_dot_data(idx:end);
            theta_p_dot_kf_data = theta_p_dot_kf_data(idx:end);
            theta_w_data = theta_w_data(idx:end);
            theta_w_kf_data = theta_w_kf_data(idx:end);
            theta_w_dot_data = theta_w_dot_data(idx:end);
            theta_w_dot_kf_data = theta_w_dot_kf_data(idx:end);
            log_motor_value_data = log_motor_value_data(idx:end);
            log_pwm_duty_data = log_pwm_duty_data(idx:end);
            log_motor_direction_data = log_motor_direction_data(idx:end);
        end
        
        %% サブプロットの更新
        for i = 1:6
            if elapsed_time > time_window
                set(ax(i), 'XLim', [elapsed_time - time_window, elapsed_time]);
            else
                set(ax(i), 'XLim', [0, time_window]);
            end
        end
        
        % プロットの更新
        set(theta_p_plot, 'XData', time_data, 'YData', theta_p_data);
        set(theta_p_kf_plot, 'XData', time_data, 'YData', theta_p_kf_data);
        set(theta_p_dot_plot, 'XData', time_data, 'YData', theta_p_dot_data);
        set(theta_p_dot_kf_plot, 'XData', time_data, 'YData', theta_p_dot_kf_data);
        set(theta_w_plot, 'XData', time_data, 'YData', theta_w_data);
        set(theta_w_kf_plot, 'XData', time_data, 'YData', theta_w_kf_data);
        set(theta_w_dot_plot, 'XData', time_data, 'YData', theta_w_dot_data);
        set(theta_w_dot_kf_plot, 'XData', time_data, 'YData', theta_w_dot_kf_data);
        set(log_motor_value_plot, 'XData', time_data, 'YData', log_motor_value_data);
        set(log_pwm_duty_plot, 'XData', time_data, 'YData', log_pwm_duty_data);
        set(log_motor_direction_plot, 'XData', time_data, 'YData', log_motor_direction_data);
        
        %% 倒立振子の更新
        % 角度と位置の計算
        theta_p_deg = rad2deg(theta_p);
        theta_w_deg = rad2deg(theta_w);
        trans_x = r_wheel * theta_w;  % ホイールの回転に応じた並進
        
        % ポールの頂点を回転・並進
        R_pole = [cosd(theta_p_deg), -sind(theta_p_deg); sind(theta_p_deg), cosd(theta_p_deg)];
        pole_vertices = (R_pole * pole_shape.Vertices')';
        pole_vertices(:,1) = pole_vertices(:,1) + trans_x;
        pole_vertices(:,2) = pole_vertices(:,2) + r_wheel;
        
        % ホイールの頂点を回転・並進
        wheel_theta = linspace(0, 2*pi, 100) + theta_w;
        wheel_x = r_wheel * cos(wheel_theta) + trans_x;
        wheel_y = r_wheel * sin(wheel_theta);
        
        % ホイールの目印ラインの座標を回転・並進
        R_wheel = [cos(theta_w), -sin(theta_w); sin(theta_w), cos(theta_w)];
        line_coords = R_wheel * [line_wheel_x; line_wheel_y];
        line_coords(1,:) = line_coords(1,:) + trans_x;
        line_coords(2,:) = line_coords(2,:);
        
        % プロットの更新
        set(h_pole, 'XData', pole_vertices(:,1), 'YData', pole_vertices(:,2));
        set(h_wheel, 'XData', wheel_x, 'YData', wheel_y + r_wheel);
        set(h_line_wheel, 'XData', line_coords(1,:), 'YData', line_coords(2,:) + r_wheel);
        
        drawnow;
        
    else
        % データがない場合は短時間待機
        pause(0.01);
    end
end
