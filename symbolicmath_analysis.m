clear; clc; close all;

% --- 1. กำหนดค่าคงที่ (Parameters) ---
L0 = 4; 
L1 = 2; 
L2 = 2; 
L3 = 2; 
L4 = 2.4; 
L5 = 2.4;

% จุดยึด Ground
A = [-L0/2, 0]; 
E = [L0/2, 0];

% --- 2. ตั้งค่ากราฟิก (Setup) ---
figure('Color', 'w', 'Name', 'Roberts Mechanism Simulation Display');
axis equal; grid on; hold on;

% กำหนดขอบเขตแกนที่นี่ครั้งเดียว
axis([-4 4 -5 3]); 
xlabel('X Position'); ylabel('Y Position');
title('Roberts Mechanism Analysis');

% วาดจุดยึด A, E
plot(A(1), A(2), 'k^', 'MarkerSize', 12, 'MarkerFaceColor', 'k');
plot(E(1), E(2), 'k^', 'MarkerSize', 12, 'MarkerFaceColor', 'k');
text(A(1)-0.4, A(2)-0.3, 'A');
text(E(1)+0.2, E(2)-0.3, 'E');

% เตรียมเส้นกราฟ
h_Link1  = plot(nan, nan, 'b-', 'LineWidth', 2); 
h_Link3  = plot(nan, nan, 'b-', 'LineWidth', 2); 
h_Link2  = plot(nan, nan, 'k-', 'LineWidth', 2); 
h_Tri    = plot(nan, nan, 'r-', 'LineWidth', 2); 
h_Joints = plot(nan, nan, 'ko', 'MarkerFaceColor', 'w', 'MarkerSize', 6);
h_PointC = plot(nan, nan, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 8);
h_Trace  = plot(nan, nan, 'g--', 'LineWidth', 1.5);

% สร้างกล่องข้อความสำหรับแสดงผล (มุมบนซ้าย)
h_Text = text(-3.5, 2.5, '', ...
    'FontSize', 12, ...
    'FontName', 'Courier New', ... % ใช้ Font แบบ Fixed width ตัวเลขจะได้ไม่ดิ้น
    'BackgroundColor', 'w', ...    % พื้นหลังสีขาวจะได้อ่านง่าย
    'EdgeColor', 'k');             % ขอบสีดำ

% --- 3. เริ่ม Animation ---
t_range = linspace(deg2rad(28.9), deg2rad(75.5), 60); % เพิ่มจำนวนเฟรมให้นุ่มนวลขึ้น
theta_seq = [t_range, fliplr(t_range)]; 

% Preallocation
total_steps = length(theta_seq);
trace_x_data = nan(1, total_steps);
trace_y_data = nan(1, total_steps);
counter = 0;

disp('Starting Animation...');

for i = 1:total_steps
    th = theta_seq(i);

    % 1. หาพิกัด B
    Bx = A(1) + L1 * cos(th);
    By = A(2) + L1 * sin(th);

    % 2. หาพิกัด D
    [Dx_all, Dy_all] = my_circle_intersection(Bx, By, L2, E(1), E(2), L3);
    if isempty(Dx_all), continue; end
    [~, idx_D] = max(Dy_all);
    Dx = Dx_all(idx_D); Dy = Dy_all(idx_D);

    % 3. หาพิกัด C
    [Cx_all, Cy_all] = my_circle_intersection(Bx, By, L4, Dx, Dy, L5);
    if isempty(Cx_all), continue; end
    [~, idx_C] = min(Cy_all);
    Cx = Cx_all(idx_C); Cy = Cy_all(idx_C);

    % เก็บข้อมูล Trace
    counter = counter + 1;
    trace_x_data(counter) = Cx;
    trace_y_data(counter) = Cy;

    % --- 4. อัปเดตกราฟ ---
    set(h_Link1, 'XData', [A(1), Bx], 'YData', [A(2), By]);
    set(h_Link3, 'XData', [E(1), Dx], 'YData', [E(2), Dy]);
    set(h_Link2, 'XData', [Bx, Dx],   'YData', [By, Dy]);
    set(h_Tri,   'XData', [Bx, Cx, Dx, Bx], 'YData', [By, Cy, Dy, By]);
    set(h_Joints,'XData', [Bx, Dx], 'YData', [By, Dy]);
    set(h_PointC,'XData', Cx, 'YData', Cy);
    set(h_Trace, 'XData', trace_x_data(1:counter), 'YData', trace_y_data(1:counter));

    % แปลงมุมเป็นองศา (rad2deg)
    info_str = sprintf('Input Angle: %.2f deg\nTrace X:     %.4f', rad2deg(th), Cx);
    set(h_Text, 'String', info_str);

    drawnow;
    pause(0.05); 
end

disp('Animation Finished.');

% --- Helper Function ---
function [x_out, y_out] = my_circle_intersection(x1, y1, r1, x2, y2, r2)
    d2 = (x1 - x2)^2 + (y1 - y2)^2;
    d = sqrt(d2);
    if d > r1 + r2 || d < abs(r1 - r2) || d == 0
        x_out = []; y_out = []; return;
    end
    a = (r1^2 - r2^2 + d2) / (2 * d);
    h = sqrt(max(0, r1^2 - a^2));
    x0 = x1 + a * (x2 - x1) / d;
    y0 = y1 + a * (y2 - y1) / d;
    rx = -(y2 - y1) * (h / d);
    ry = -(x2 - x1) * (h / d);
    x_out = [x0 + rx, x0 - rx];
    y_out = [y0 - ry, y0 + ry];
end