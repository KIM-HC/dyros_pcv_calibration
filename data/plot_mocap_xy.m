clc
clear all

% load data
% Time center:X center:Y center:Z x_axis:X x_axis:Y x_axis:Z y_axis:X y_axis:Y y_axis:Z
date_num = "2022_03_07_";
set_num = 3;
% mv = load("mocap/equal_mocap_" + date_num + set_num + ".txt");
mv = load("mocap/mocap_" + date_num + set_num + ".txt");
% mv = load("simulation/mocap_" + date_num + set_num + ".csv");

center_idx = 5;

% plot path
figure(11)
subplot(1,1,1)
plot(mv(:,1),mv(:,center_idx), 'LineWidth',1)
hold on
plot(mv(:,1),mv(:,center_idx+1), 'LineWidth',1)
hold off
legend({'X','Y'},'Location','best')
title("set" + set_num + " mocap data")
grid on
