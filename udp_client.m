%% EtherSpin-ESP MATLAB UDP Client Sample
% ステッピングモータをMATLABから制御するサンプルコード
%
% 使用方法:
%   1. ESP32のIPアドレスを設定
%   2. セクションごとに実行（Ctrl+Enter）

%% 初期設定
clear all;
close all;
clc;

% ESP32設定
ESP32_IP = 'motor.local';  % またはIPアドレス (例: '192.168.1.100')
UDP_PORT = 8888;

% UDPオブジェクト作成
u = udpport("datagram", "IPV4");

fprintf('EtherSpin-ESP MATLAB Controller\n');
fprintf('Target: %s:%d\n', ESP32_IP, UDP_PORT);

%% 関数定義: 速度設定
function setSpeed(u, ip, port, speed)
    % 速度を設定（step/s）
    message = sprintf('{"v": %.2f}', speed);
    write(u, uint8(message), "string", ip, port);
end

%% 例1: 定速回転
fprintf('\n=== Example 1: Constant Speed ===\n');

% 500 step/s で5秒間回転
fprintf('Running at 500 step/s for 5 seconds...\n');
setSpeed(u, ESP32_IP, UDP_PORT, 500);
pause(5);

% 停止
fprintf('Stopping...\n');
setSpeed(u, ESP32_IP, UDP_PORT, 0);
pause(1);

fprintf('Done!\n');

%% 例2: 台形加速プロファイル
fprintf('\n=== Example 2: Trapezoidal Acceleration ===\n');

duration = 10;      % 秒
maxSpeed = 2000;    % step/s
dt = 0.05;          % 更新間隔（秒）

t = 0:dt:duration;
speed = zeros(size(t));

for i = 1:length(t)
    if t(i) < duration/3
        % 加速
        speed(i) = maxSpeed * (t(i) / (duration/3));
    elseif t(i) < 2*duration/3
        % 定速
        speed(i) = maxSpeed;
    else
        % 減速
        speed(i) = maxSpeed * (1 - (t(i) - 2*duration/3) / (duration/3));
    end
    
    setSpeed(u, ESP32_IP, UDP_PORT, speed(i));
    fprintf('Time: %.2f s, Speed: %.1f step/s\n', t(i), speed(i));
    pause(dt);
end

% 停止
setSpeed(u, ESP32_IP, UDP_PORT, 0);

% プロット
figure('Name', 'Trapezoidal Acceleration Profile');
plot(t, speed, 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Speed [step/s]');
title('Trapezoidal Acceleration Profile');
grid on;

fprintf('Done!\n');

%% 例3: サイン波速度プロファイル
fprintf('\n=== Example 3: Sine Wave Profile ===\n');

duration = 20;      % 秒
amplitude = 1500;   % step/s
frequency = 0.2;    % Hz
dt = 0.02;          % 更新間隔（秒）

t = 0:dt:duration;
speed = amplitude * sin(2*pi*frequency*t);

fprintf('Running sine wave profile (Amp: %.0f step/s, Freq: %.2f Hz)\n', ...
        amplitude, frequency);

for i = 1:length(t)
    setSpeed(u, ESP32_IP, UDP_PORT, speed(i));
    
    if mod(i, 50) == 0  % 1秒ごとに表示
        fprintf('Time: %.1f s, Speed: %.1f step/s\n', t(i), speed(i));
    end
    
    pause(dt);
end

% 停止
setSpeed(u, ESP32_IP, UDP_PORT, 0);

% プロット
figure('Name', 'Sine Wave Profile');
plot(t, speed, 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Speed [step/s]');
title('Sine Wave Velocity Profile');
grid on;

fprintf('Done!\n');

%% 例4: S字カーブ加速
fprintf('\n=== Example 4: S-Curve Acceleration ===\n');

duration = 8;       % 秒
maxSpeed = 2500;    % step/s
dt = 0.05;          % 更新間隔（秒）

t = 0:dt:duration;
speed = zeros(size(t));

fprintf('Running S-curve acceleration profile...\n');

for i = 1:length(t)
    % S字カーブ（シグモイド関数）
    x = (t(i)/duration - 0.5) * 10;
    sigmoid = 1 / (1 + exp(-x));
    speed(i) = maxSpeed * sigmoid;
    
    setSpeed(u, ESP32_IP, UDP_PORT, speed(i));
    fprintf('Time: %.2f s, Speed: %.1f step/s\n', t(i), speed(i));
    
    pause(dt);
end

% 停止
setSpeed(u, ESP32_IP, UDP_PORT, 0);

% プロット
figure('Name', 'S-Curve Acceleration');
subplot(2,1,1);
plot(t, speed, 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Speed [step/s]');
title('S-Curve Velocity Profile');
grid on;

subplot(2,1,2);
acceleration = [0, diff(speed)/dt];
plot(t, acceleration, 'LineWidth', 2, 'Color', 'r');
xlabel('Time [s]');
ylabel('Acceleration [step/s^2]');
title('Acceleration Profile');
grid on;

fprintf('Done!\n');

%% 例5: 二次関数速度プロファイル
fprintf('\n=== Example 5: Quadratic Profile ===\n');

duration = 6;       % 秒
maxSpeed = 2000;    % step/s
dt = 0.05;          % 更新間隔（秒）

t = 0:dt:duration;
speed = zeros(size(t));

fprintf('Running quadratic acceleration profile...\n');

for i = 1:length(t)
    if t(i) < duration/2
        % 二次関数で加速
        normalized_t = 2 * t(i) / duration;
        speed(i) = maxSpeed * normalized_t^2;
    else
        % 二次関数で減速
        normalized_t = 2 * (duration - t(i)) / duration;
        speed(i) = maxSpeed * normalized_t^2;
    end
    
    setSpeed(u, ESP32_IP, UDP_PORT, speed(i));
    fprintf('Time: %.2f s, Speed: %.1f step/s\n', t(i), speed(i));
    
    pause(dt);
end

% 停止
setSpeed(u, ESP32_IP, UDP_PORT, 0);

% プロット
figure('Name', 'Quadratic Profile');
plot(t, speed, 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Speed [step/s]');
title('Quadratic Velocity Profile');
grid on;

fprintf('Done!\n');

%% 例6: 往復運動
fprintf('\n=== Example 6: Reciprocating Motion ===\n');

cycles = 3;         % サイクル数
cycleTime = 4;      % 1サイクルの時間（秒）
maxSpeed = 1800;    % step/s
dt = 0.02;          % 更新間隔（秒）

totalTime = cycles * cycleTime;
t = 0:dt:totalTime;
speed = zeros(size(t));

fprintf('Running reciprocating motion (%d cycles)...\n', cycles);

for i = 1:length(t)
    % 三角波
    phase = mod(t(i), cycleTime);
    if phase < cycleTime/2
        speed(i) = maxSpeed * (2*phase/cycleTime);
    else
        speed(i) = maxSpeed * (2 - 2*phase/cycleTime);
    end
    
    setSpeed(u, ESP32_IP, UDP_PORT, speed(i));
    
    if mod(i, 50) == 0  % 1秒ごとに表示
        fprintf('Time: %.1f s, Speed: %.1f step/s\n', t(i), speed(i));
    end
    
    pause(dt);
end

% 停止
setSpeed(u, ESP32_IP, UDP_PORT, 0);

% プロット
figure('Name', 'Reciprocating Motion');
plot(t, speed, 'LineWidth', 2);
xlabel('Time [s]');
ylabel('Speed [step/s]');
title('Reciprocating Motion Profile');
grid on;

fprintf('Done!\n');

%% クリーンアップ
fprintf('\n=== Cleanup ===\n');
clear u;
fprintf('UDP connection closed.\n');
