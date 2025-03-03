close all;
clear all;

addpath(strcat(pwd), '../');
plotsettings;

% 表示する時間幅（秒）
time_window = 15;

% プロットの設定
figure;

subplot(3, 2, 1);
hold on;
xlabel('Time (s)');
ylabel('Attitude Angle (deg)');
theta_p_plot = plot(NaN, NaN, 'b', 'DisplayName', "$\theta_p$");
theta_p_kf_plot = plot(NaN, NaN, 'r', 'DisplayName', "$\theta_{p_{kf}}$");
legend;
grid on;

subplot(3, 2, 2);
hold on;
xlabel('Time (s)');
ylabel('Angular Velocity (deg/s)');
theta_p_dot_plot = plot(NaN, NaN, 'b', 'DisplayName', "$\dot \theta_p$");
theta_p_dot_kf_plot = plot(NaN, NaN, 'r', 'DisplayName', "$\dot \theta_{p_{kf}}$");
legend;
grid on;

subplot(3, 2, 3);
hold on;
xlabel('Time (s)');
ylabel('Wheel Angle (deg)');
theta_w_plot = plot(NaN, NaN, 'b', 'DisplayName', "$\theta_w$");
theta_w_kf_plot = plot(NaN, NaN, 'r', 'DisplayName', "$\theta_{w_{kf}}$");
legend;
grid on;

subplot(3, 2, 4);
hold on;
xlabel('Time (s)');
ylabel('Wheel Angular Velocity (deg/s)');
theta_w_dot_plot = plot(NaN, NaN, 'b', 'DisplayName', "$\dot \theta_w$");
theta_w_dot_kf_plot = plot(NaN, NaN, 'r', 'DisplayName', "$\dot \theta_{w_{kf}}$");
legend;
grid on;

subplot(3, 2, 5);
hold on;
xlabel('Time (s)');
ylabel('Motor Value / PWM Duty');
log_motor_value_plot = plot(NaN, NaN, 'b', 'DisplayName', "Motor Value");
log_pwm_duty_plot = plot(NaN, NaN, 'r', 'DisplayName', "PWM Duty");
legend;
grid on;

subplot(3, 2, 6);
hold on;
xlabel('Time (s)');
ylabel('Motor Direction');
log_motor_direction_plot = plot(NaN, NaN, 'b', 'DisplayName', "Motor Direction");
legend;
grid on;

% データ保存用の変数
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

drawnow;

% UDP受信ポートの設定
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
            
            % グラフのx軸をずらす
            for i = 1:6
                subplot(3, 2, i);
                xlim([elapsed_time - time_window, elapsed_time]);
            end
        else
            for i = 1:6
                subplot(3, 2, i);
                xlim([0, time_window]);
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
        
        drawnow;
        
    else
        % disp('No data available');
        % pause(1); % 1秒待つ
    end
end
