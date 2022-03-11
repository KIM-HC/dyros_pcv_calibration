clc
clear all

% load data
% Time center:X center:Y center:Z x_axis:X x_axis:Y x_axis:Z y_axis:X y_axis:Y y_axis:Z
date_num_mv = "2022_03_07_3";
date_num_jt = "2022_03_07_25";
mv = load("mocap/mocap_" + date_num_mv + ".txt");
jt = load("joint/joint_" + date_num_jt + ".csv");

% plot path
figure(11)
subplot(1,1,1)
plot(mv(:,2), 'LineWidth',1)
hold on
plot(mv(:,3), 'LineWidth',1)
hold off
legend({'X','Y'},'Location','best')
title('mocap data')
grid on

figure(12)
subplot(1,1,1)
plot(jt(:,2), 'LineWidth',1)
title('jt data rot')
hold on
plot(jt(:,4), 'LineWidth',1)
plot(jt(:,6), 'LineWidth',1)
plot(jt(:,8), 'LineWidth',1)
legend({'set 0','set 1','set 2','set 3'},'Location','best')
grid on
hold off

% figure(13)
% subplot(1,1,1)
% plot(jt(jt_st:jt_ed,3), 'LineWidth',1)
% title('jt data steer')
% hold on
% plot(jt(jt_st:jt_ed,5), 'LineWidth',1)
% plot(jt(jt_st:jt_ed,7), 'LineWidth',1)
% plot(jt(jt_st:jt_ed,9), 'LineWidth',1)
% legend({'set 0','set 1','set 2','set 3'},'Location','best')
% grid on
% hold off

