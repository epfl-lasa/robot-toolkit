clc
clear
% close all
figure
Shift=5;
Data=importfile('Position_Gain_0.7_Amp_0.4.txt');
axes1 = subplot(3,1,1);
hold(axes1,'on');
xlim(axes1,[0 20])
plot(Data(1:end-Shift,15),Data(1:end-Shift,1),'LineWidth',3,'Color',[1 0 0],'DisplayName','The Real Position of the First Joint')
hold on
plot(Data(1:end-Shift,15),Data(Shift+1:end,8),'LineWidth',3,'Color',[0 0 1],'DisplayName','The Desired Position of the First Joint')
xlabel('Time [s]');
ylabel('Joint Position [rad]');
box(axes1,'on');
grid(axes1,'on');
set(axes1,'FontSize',22);
legend(axes1,'show');



axes1 = subplot(3,1,2);
xlim(axes1,[0 20])
hold(axes1,'on');
plot(Data(1:end-Shift,15),Data(1:end-Shift,7),'LineWidth',3,'Color',[1 0 0],'DisplayName','The Real Position of the Seventh Joint')
hold on
plot(Data(1:end-Shift,15),Data(Shift+1:end,14),'LineWidth',3,'Color',[0 0 1],'DisplayName','The Real Position of the Seventh Joint')
hold on
xlabel('Time [s]');
ylabel('Joint Position [rad]');
box(axes1,'on');
grid(axes1,'on');
set(axes1,'FontSize',22);
legend(axes1,'show');

axes1 = subplot(3,1,3);
hold(axes1,'on');
xlim(axes1,[2 20])
plot(Data(1:end-Shift,15),abs(Data(1:end-Shift,1)-Data(Shift+1:end,8)),'LineWidth',3,'DisplayName','The Tracking Error of the First Joint')
hold on
plot(Data(1:end-Shift,15),abs(Data(1:end-Shift,2)-Data(Shift+1:end,9)),'LineWidth',3,'DisplayName','The Tracking Error of the Second Joint')
hold on
plot(Data(1:end-Shift,15),abs(Data(1:end-Shift,3)-Data(Shift+1:end,10)),'LineWidth',3,'DisplayName','The Tracking Error of the Third Joint')
hold on
plot(Data(1:end-Shift,15),abs(Data(1:end-Shift,4)-Data(Shift+1:end,11)),'LineWidth',3,'DisplayName','The Tracking Error of the Fourth Joint')
hold on
plot(Data(1:end-Shift,15),abs(Data(1:end-Shift,5)-Data(Shift+1:end,12)),'LineWidth',3,'DisplayName','The Tracking Error of the Fifth Joint')
hold on
plot(Data(1:end-Shift,15),abs(Data(1:end-Shift,6)-Data(Shift+1:end,13)),'LineWidth',3,'DisplayName','The Tracking Error of the Sixth Joint')
hold on
plot(Data(1:end-Shift,15),abs(Data(1:end-Shift,7)-Data(Shift+1:end,14)),'LineWidth',3,'DisplayName','The Tracking Error of the Seventh Joint')
hold on
xlabel('Time [s]');
ylabel('Tracking Error [rad]');
box(axes1,'on');
grid(axes1,'on');
set(axes1,'FontSize',22);
legend(axes1,'show');
