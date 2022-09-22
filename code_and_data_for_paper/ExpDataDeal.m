%处理实验数据
clear all
clc

%% 获取实验数据
angle1 = readmatrix('forward.txt');
angle2 = readmatrix('lateral.txt');
angle3 = readmatrix('turn.txt');
input_angle  = readmatrix('inputTraj.txt');
r1 = input_angle(:,3);
r2 = input_angle(:,6);
r1 = r1(1500:end);
r2 = -r2(1500:end);
t_input = 0.001:0.001:size(r1,1)/1000;

t=0.001:0.001:size(angle1(:,1))/1000; %1ms执行一次
t1=t(1:end-1);
t2=t1(1:end-1);

%三种步态下，腿1和腿2的行走参数
angle1_1 = angle1(:,1);
angle1_2 = angle1(:,2);
angle1_3 = angle1(:,3);
angle1_4 = angle1(:,4);
angle1_5 = angle1(:,5);
angle1_6 = angle1(:,6);

angle2_1 = angle2(:,1);
angle2_2 = angle2(:,2);
angle2_3 = angle2(:,3);
angle2_4 = angle2(:,4);
angle2_5 = angle2(:,5);
angle2_6 = angle2(:,6);

angle3_1 = angle3(:,1);
angle3_2 = angle3(:,2);
angle3_3 = angle3(:,3);
angle3_4 = angle3(:,4);
angle3_5 = angle3(:,5);
angle3_6 = angle3(:,6);

%一次差分
d_angle1_1=(angle1_1(2:end)-angle1_1(1:end-1))*1000;
d_angle1_2=(angle1_2(2:end)-angle1_2(1:end-1))*1000;
d_angle1_3=(angle1_3(2:end)-angle1_3(1:end-1))*1000;
d_angle1_4=(angle1_4(2:end)-angle1_4(1:end-1))*1000;
d_angle1_5=(angle1_5(2:end)-angle1_5(1:end-1))*1000;
d_angle1_6=(angle1_6(2:end)-angle1_6(1:end-1))*1000;

d_angle2_1=(angle2_1(2:end)-angle2_1(1:end-1))*1000;
d_angle2_2=(angle2_2(2:end)-angle2_2(1:end-1))*1000;
d_angle2_3=(angle2_3(2:end)-angle2_3(1:end-1))*1000;
d_angle2_4=(angle2_4(2:end)-angle2_4(1:end-1))*1000;
d_angle2_5=(angle2_5(2:end)-angle2_5(1:end-1))*1000;
d_angle2_6=(angle2_6(2:end)-angle2_6(1:end-1))*1000;

d_angle3_1=(angle3_1(2:end)-angle3_1(1:end-1))*1000;
d_angle3_2=(angle3_2(2:end)-angle3_2(1:end-1))*1000;
d_angle3_3=(angle3_3(2:end)-angle3_3(1:end-1))*1000;
d_angle3_4=(angle3_4(2:end)-angle3_4(1:end-1))*1000;
d_angle3_5=(angle3_5(2:end)-angle3_5(1:end-1))*1000;
d_angle3_6=(angle3_6(2:end)-angle3_6(1:end-1))*1000;

%二次差分
dd_angle1_1=(d_angle1_1(2:end)-d_angle1_1(1:end-1))*1000;
dd_angle1_2=(d_angle1_2(2:end)-d_angle1_2(1:end-1))*1000;
dd_angle1_3=(d_angle1_3(2:end)-d_angle1_3(1:end-1))*1000;
dd_angle1_4=(d_angle1_4(2:end)-d_angle1_4(1:end-1))*1000;
dd_angle1_5=(d_angle1_5(2:end)-d_angle1_5(1:end-1))*1000;
dd_angle1_6=(d_angle1_6(2:end)-d_angle1_6(1:end-1))*1000;

dd_angle2_1=(d_angle2_1(2:end)-d_angle2_1(1:end-1))*1000;
dd_angle2_2=(d_angle2_2(2:end)-d_angle2_2(1:end-1))*1000;
dd_angle2_3=(d_angle2_3(2:end)-d_angle2_3(1:end-1))*1000;
dd_angle2_4=(d_angle2_4(2:end)-d_angle2_4(1:end-1))*1000;
dd_angle2_5=(d_angle2_5(2:end)-d_angle2_5(1:end-1))*1000;
dd_angle2_6=(d_angle2_6(2:end)-d_angle2_6(1:end-1))*1000;

dd_angle3_1=(d_angle3_1(2:end)-d_angle3_1(1:end-1))*1000;
dd_angle3_2=(d_angle3_2(2:end)-d_angle3_2(1:end-1))*1000;
dd_angle3_3=(d_angle3_3(2:end)-d_angle3_3(1:end-1))*1000;
dd_angle3_4=(d_angle3_4(2:end)-d_angle3_4(1:end-1))*1000;
dd_angle3_5=(d_angle3_5(2:end)-d_angle3_5(1:end-1))*1000;
dd_angle3_6=(d_angle3_6(2:end)-d_angle3_6(1:end-1))*1000;

%% 画图 前进方向

%位置图
h1 = figure;
figure(h1)
tiledlayout(2,2); %两条腿的速度和加速度
set(gcf,'Units','centimeters','Position',[5 5 28 14]); %指定plot输出图片的尺寸，xmin，ymin，width，height



nexttile
plot(t1,d_angle1_1,'r',t1,d_angle1_2,'g',t1,d_angle1_3,'b');
xlabel('t(s)');
ylabel('dq(rad/s)');
title('leg1 motors vel');
ylim([-350 350]);
set(gca,'YTick',[-300:300:300]);
ax = gca;
ax.TitleHorizontalAlignment = 'right';
set(gca,'FontName','Times new Roman','FontSize',20);

nexttile
plot(t1,d_angle1_4,'r',t1,d_angle1_5,'g',t1,d_angle1_6,'b');
xlabel('t(s)');
ylabel('dq(rad/s)');
title('leg2 motors vel');
ylim([-350 350]);
set(gca,'YTick',[-300:300:300]);
set(gca,'FontName','Times new Roman','FontSize',20);
ax = gca;
ax.TitleHorizontalAlignment = 'right';

nexttile
plot(t2,dd_angle1_1,'r',t2,dd_angle1_2,'g',t2,dd_angle1_3,'b');
xlabel('t(s)');
ylabel('ddq(rad/s^2)');
title('leg1 motors acc');
ylim([-80000 80000]);
set(gca,'YTick',[-50000:50000:50000]);
set(gca,'FontName','Times new Roman','FontSize',20);
ax = gca;
ax.TitleHorizontalAlignment = 'right';

nexttile
plot(t2,dd_angle1_4,'r',t2,dd_angle1_5,'g',t2,dd_angle1_6,'b');
xlabel('t(s)');
ylabel('ddq(rad/s^2)');
title('leg2 motors acc');
ylim([-80000 80000]);
set(gca,'YTick',[-50000:50000:50000]);
set(gca,'FontName','Times new Roman','FontSize',20);
ax = gca;
ax.TitleHorizontalAlignment = 'right';

lgd = legend('x','y','r','Orientation','vertical');
legend('boxoff');
lgd.Layout.Tile = 'east';
set(lgd,'FontSize',28)






%% 画图 侧移方向

%位置图
h2 = figure;
figure(h2)
tiledlayout(2,2); %两条腿的速度和加速度
set(gcf,'Units','centimeters','Position',[5 5 28 14]); %指定plot输出图片的尺寸，xmin，ymin，width，height


nexttile
plot(t1,d_angle2_1,'r',t1,d_angle2_2,'g',t1,d_angle2_3,'b');
xlabel('t(s)');
ylabel('dq(rad/s)');
title('leg1 motors vel');
ylim([-350 350]);
set(gca,'YTick',[-300:300:300]);
ax = gca;
ax.TitleHorizontalAlignment = 'right';
set(gca,'FontName','Times new Roman','FontSize',20);

nexttile
plot(t1,d_angle2_4,'r',t1,d_angle2_5,'g',t1,d_angle2_6,'b');
xlabel('t(s)');
ylabel('dq(rad/s)');
title('leg2 motors vel');
ylim([-350 350]);
set(gca,'YTick',[-300:300:300]);
set(gca,'FontName','Times new Roman','FontSize',20);
ax = gca;
ax.TitleHorizontalAlignment = 'right';

nexttile
plot(t2,dd_angle2_1,'r',t2,dd_angle2_2,'g',t2,dd_angle2_3,'b');
xlabel('t(s)');
ylabel('ddq(rad/s^2)');
title('leg1 motors acc');
ylim([-80000 80000]);
set(gca,'YTick',[-50000:50000:50000]);
set(gca,'FontName','Times new Roman','FontSize',20);
ax = gca;
ax.TitleHorizontalAlignment = 'right';

nexttile
plot(t2,dd_angle2_4,'r',t2,dd_angle2_5,'g',t2,dd_angle2_6,'b');
xlabel('t(s)');
ylabel('ddq(rad/s^2)');
title('leg2 motors acc');
ylim([-80000 80000]);
set(gca,'YTick',[-50000:50000:50000]);
set(gca,'FontName','Times new Roman','FontSize',20);
ax = gca;
ax.TitleHorizontalAlignment = 'right';

lgd = legend('x','y','r','Orientation','vertical');
legend('boxoff');
lgd.Layout.Tile = 'east';
set(lgd,'FontSize',28)


%% 画图 旋转

%位置图
h3 = figure;
figure(h3)
tiledlayout(2,2); %两条腿的速度和加速度
set(gcf,'Units','centimeters','Position',[5 5 28 14]); %指定plot输出图片的尺寸，xmin，ymin，width，height


nexttile
plot(t1,d_angle3_1,'r',t1,d_angle3_2,'g',t1,d_angle3_3,'b');
xlabel('t(s)');
ylabel('dq(rad/s)');
title('leg1 motors vel');
ylim([-350 350]);
set(gca,'YTick',[-300:300:300]);
ax = gca;
ax.TitleHorizontalAlignment = 'right';
set(gca,'FontName','Times new Roman','FontSize',20);

nexttile
plot(t1,d_angle3_4,'r',t1,d_angle3_5,'g',t1,d_angle3_6,'b');
xlabel('t(s)');
ylabel('dq(rad/s)');
title('leg2 motors vel');
ylim([-350 350]);
set(gca,'YTick',[-300:300:300]);
set(gca,'FontName','Times new Roman','FontSize',20);
ax = gca;
ax.TitleHorizontalAlignment = 'right';

nexttile
plot(t2,dd_angle3_1,'r',t2,dd_angle3_2,'g',t2,dd_angle3_3,'b');
xlabel('t(s)');
ylabel('ddq(rad/s^2)');
title('leg1 motors acc');
ylim([-80000 80000]);
set(gca,'YTick',[-50000:50000:50000]);
set(gca,'FontName','Times new Roman','FontSize',20);
ax = gca;
ax.TitleHorizontalAlignment = 'right';

nexttile
plot(t2,dd_angle3_4,'r',t2,dd_angle3_5,'g',t2,dd_angle3_6,'b');
xlabel('t(s)');
ylabel('ddq(rad/s^2)');
title('leg2 motors acc');
ylim([-80000 80000]);
set(gca,'YTick',[-50000:50000:50000]);
set(gca,'FontName','Times new Roman','FontSize',20);
ax = gca;
ax.TitleHorizontalAlignment = 'right';

lgd = legend('x','y','r','Orientation','vertical');
legend('boxoff');
lgd.Layout.Tile = 'east';
set(lgd,'FontSize',28)


%% 换一种画法，6*2  一组速度，一组加速度
%速度
h4 = figure;
figure(h4)
tiledlayout(2,3); %两条腿的速度
set(gcf,'Units','centimeters','Position',[5 5 28 14]); %指定plot输出图片的尺寸，xmin，ymin，width，height


nexttile  %1
plot(t1,d_angle1_1,'r',t1,d_angle1_2,'g',t1,d_angle1_3,'b');
xlabel('t(s)');
ylabel("leg1 motor (rad/s)");
title('Foward','Position',[5,210])
ylim([-350 350]);
set(gca,'YTick',[-300:300:300]);
set(gca,'XTick',[0:2.5:5]);
ax = gca;
ax.TitleHorizontalAlignment = 'right';
set(gca,'FontName','Times new Roman','FontSize',20);

nexttile  %2
plot(t1,d_angle2_1,'r',t1,d_angle2_2,'g',t1,d_angle2_3,'b');
xlabel('t(s)');
ylabel("leg1 motor (rad/s)");
title('Lateral','Position',[5,210])
ylim([-350 350]);
set(gca,'YTick',[-300:300:300]);
set(gca,'XTick',[0:2.5:5]);
ax = gca;
ax.TitleHorizontalAlignment = 'right';
set(gca,'FontName','Times new Roman','FontSize',20);

nexttile  %3
plot(t1,d_angle3_1,'r',t1,d_angle3_2,'g',t1,d_angle3_3,'b');
xlabel('t(s)');
ylabel("leg1 motor (rad/s)");
title('Pivot','Position',[5,210])
ylim([-350 350]);
set(gca,'YTick',[-300:300:300]);
set(gca,'XTick',[0:2.5:5]);
ax = gca;
ax.TitleHorizontalAlignment = 'right';
set(gca,'FontName','Times new Roman','FontSize',20);

nexttile  %4
plot(t1,d_angle1_4,'r',t1,d_angle1_5,'g',t1,d_angle1_6,'b');
xlabel('t(s)');
ylabel("leg2 motor (rad/s)");
title('Foward','Position',[5,210])
ylim([-350 350]);
set(gca,'YTick',[-300:300:300]);
set(gca,'XTick',[0:2.5:5]);
ax = gca;
ax.TitleHorizontalAlignment = 'right';
set(gca,'FontName','Times new Roman','FontSize',20);

nexttile %5
plot(t1,d_angle2_4,'r',t1,d_angle2_5,'g',t1,d_angle2_6,'b');
xlabel('t(s)');
ylabel("leg2 motor (rad/s)");
title('Lateral','Position',[5,210])
ylim([-350 350]);
set(gca,'YTick',[-300:300:300]);
set(gca,'XTick',[0:2.5:5]);
ax = gca;
ax.TitleHorizontalAlignment = 'right';
set(gca,'FontName','Times new Roman','FontSize',20);

nexttile %6
plot(t1,d_angle3_4,'r',t1,d_angle3_5,'g',t1,d_angle3_6,'b');
xlabel('t(s)');
ylabel("leg2 motor (rad/s)");
title('Pivot','Position',[5,210])
ylim([-350 350]);
set(gca,'YTick',[-300:300:300]);
set(gca,'XTick',[0:2.5:5]);
ax = gca;
ax.TitleHorizontalAlignment = 'right';
set(gca,'FontName','Times new Roman','FontSize',20);

lgd = legend('x','y','r','Orientation','vertical');
legend('boxon');
lgd.Layout.Tile = 'east';
set(lgd,'FontSize',28)


% 加速度
h5 = figure;
figure(h5)
tiledlayout(2,3); %两条腿的速度
set(gcf,'Units','centimeters','Position',[5 5 28 14]); %指定plot输出图片的尺寸，xmin，ymin，width，height


nexttile  %1
plot(t2,dd_angle1_1,'r',t2,dd_angle1_2,'g',t2,dd_angle1_3,'b');
xlabel('t(s)');
ylabel('leg1 motor (rad/s^2)');
title('Forward','Position',[5,48000])
ylim([-80000 80000]);
set(gca,'YTick',[-50000:50000:50000]);
set(gca,'XTick',[0:2.5:5]);
set(gca,'FontName','Times new Roman','FontSize',20);
ax = gca;
ax.TitleHorizontalAlignment = 'right';
% ax.YAxis.Exponent=0;%常数2为指数值，改为0即不使用科学计数法

nexttile  %2
plot(t2,dd_angle2_1,'r',t2,dd_angle2_2,'g',t2,dd_angle2_3,'b');
xlabel('t(s)');
ylabel('leg1 motor (rad/s^2)');
title('Lateral','Position',[5,48000])
ylim([-80000 80000]);
set(gca,'YTick',[-50000:50000:50000]);
set(gca,'XTick',[0:2.5:5]);
set(gca,'FontName','Times new Roman','FontSize',20);
ax = gca;
ax.TitleHorizontalAlignment = 'right';
% ax.YAxis.Exponent=0;%常数2为指数值，改为0即不使用科学计数法

nexttile  %3
plot(t2,dd_angle3_1,'r',t2,dd_angle3_2,'g',t2,dd_angle3_3,'b');
xlabel('t(s)');
ylabel('leg1 motor (rad/s^2)');
title('Pivot','Position',[5,48000])
ylim([-80000 80000]);
set(gca,'YTick',[-50000:50000:50000]);
set(gca,'XTick',[0:2.5:5]);
set(gca,'FontName','Times new Roman','FontSize',20);
ax = gca;
ax.TitleHorizontalAlignment = 'right';
% ax.YAxis.Exponent=0;%常数2为指数值，改为0即不使用科学计数法

nexttile  %4
plot(t2,dd_angle1_4,'r',t2,dd_angle1_5,'g',t2,dd_angle1_6,'b');
xlabel('t(s)');
ylabel('leg2 motor (rad/s^2)');
title('Forward','Position',[5,48000])
ylim([-80000 80000]);
set(gca,'YTick',[-50000:50000:50000]);
set(gca,'XTick',[0:2.5:5]);
set(gca,'FontName','Times new Roman','FontSize',20);
ax = gca;
ax.TitleHorizontalAlignment = 'right';
% ax.YAxis.Exponent=0;%常数2为指数值，改为0即不使用科学计数法

nexttile %5
plot(t2,dd_angle2_4,'r',t2,dd_angle2_5,'g',t2,dd_angle2_6,'b');
xlabel('t(s)');
ylabel('leg2 motor (rad/s^2)');
title('Lateral','Position',[5,48000])
ylim([-80000 80000]);
set(gca,'YTick',[-50000:50000:50000]);
set(gca,'XTick',[0:2.5:5]);
set(gca,'FontName','Times new Roman','FontSize',20);
ax = gca;
ax.TitleHorizontalAlignment = 'right';
% ax.YAxis.Exponent=0;%常数2为指数值，改为0即不使用科学计数法

nexttile %6
plot(t2,dd_angle3_4,'r',t2,dd_angle3_5,'g',t2,dd_angle3_6,'b');
xlabel('t(s)');
ylabel('leg2 motor (rad/s^2)');
title('Pivot','Position',[5,48000])
ylim([-80000 80000]);
set(gca,'YTick',[-50000:50000:50000]);
set(gca,'XTick',[0:2.5:5]);
set(gca,'FontName','Times new Roman','FontSize',20);
ax = gca;
ax.TitleHorizontalAlignment = 'right';
% ax.YAxis.Exponent=0;%常数2为指数值，改为0即不使用科学计数法

lgd = legend('x','y','r','Orientation','vertical');
legend('boxon');
lgd.Layout.Tile = 'east';
set(lgd,'FontSize',28)


%% 画电机位置
h6=figure;
figure(h6);
tiledlayout(1,2); %两条腿的速度
set(gcf,'Units','centimeters','Position',[5 5 28 14]); %指定plot输出图片的尺寸，xmin，ymin，width，height

nexttile  %1
plot(t,angle1_1,t,angle1_2,t,angle1_3);
hold on;
plot(t,angle2_1,t,angle2_2,t,angle2_3);
hold on;
plot(t,angle3_1,t,angle3_2,t,angle3_3);
xlabel('t(s)');
ylabel('leg1 motor (rad)');
% title('Lateral','Position',[5,48000])
% ylim([-80000 80000]);
% set(gca,'YTick',[-50000:50000:50000]);
xlim([0,5]);
set(gca,'XTick',[0:2.5:5]);
set(gca,'FontName','Times new Roman','FontSize',20);

% ax.TitleHorizontalAlignment = 'right';
% ax.YAxis.Exponent=0;%常数2为指数值，改为0即不使用科学计数法

nexttile  %2
plot(t,angle1_4,t,angle1_5,t,angle1_6);
hold on;
plot(t,angle2_4,t,angle2_5,t,angle2_6);
hold on;
plot(t,angle3_4,t,angle3_5,t,angle3_6);
xlabel('t(s)');
ylabel('leg2 motor (rad)');
% title('Lateral','Position',[5,48000])
% ylim([-80000 80000]);
% set(gca,'YTick',[-50000:50000:50000]);
xlim([0,5]);
set(gca,'XTick',[0:2.5:5]);
set(gca,'FontName','Times new Roman','FontSize',20);

ax = gca;
lgd=legend('F-x','F-y','F-r','L-x','L-y','L-z','P-x','P-y','P-z');
legend('boxon');
lgd.Layout.Tile = 'east';
set(lgd,'FontSize',28)

% ax.TitleHorizontalAlignment = 'right';
% ax.YAxis.Exponent=0;%常数2为指数值，改为0即不使用科学计数法


% figure;
% mark_angle2_1 = angle2_1(1:20:length(angle2_1));
% t_mark = t(1:20:length(angle2_1));
% plot(t_mark,mark_angle2_1,'-s');


%% 处理excel数据
% forward = forward(65:end,:);
% forward = table2array(forward);
% for i=1:size(forward,1)
%     forward(i,1) = forward(i,1)-1.5;
% end




%% 画仿真中的数据
load('forwardSimulationData.mat');


screwpos1 =nan(size(angle1(:,1),1),3);
screwpos2 =nan(size(angle1(:,1),1),3);
for i=1:size(angle1(:,1),1)
   ans1 = calScrewPos(angle1_1(i),angle1_2(i),angle1_3(i));
   screwpos1(i,:) = ans1;
   ans2 = calScrewPos(angle1_4(i),angle1_5(i),angle1_6(i));
   screwpos2(i,:) = ans2;
end

ts=0.001:0.001:size(angle1(:,1),1)/1000;
temp1  =screwpos1(1,1)-forward(1,2);
temp2  =screwpos1(1,2)-forward(1,3);
temp3  =screwpos1(1,3)-forward(1,4);
temp4  =screwpos2(1,1)-forward(1,5);
temp5  =screwpos2(1,2)-forward(1,6);
temp6  =screwpos2(1,3)-forward(1,7);
temp7 = screwpos2(1,3)-r2(1);
for i=1:size(forward,1)
    forward(i,2) = forward(i,2)+temp1;
    forward(i,3) = forward(i,3)+temp2;
    forward(i,4) = forward(i,4)+temp3;
    forward(i,5) = forward(i,5)+temp4;
    forward(i,6) = forward(i,6)+temp5;
    forward(i,7) = forward(i,7)+temp6;
end


for i=1:size(screwpos2,1)
    screwpos2(i,3) = screwpos2(i,3)-temp7;

end

h7 = figure;
figure(h7);
tiledlayout(1,2);
set(gcf,'Units','centimeters','Position',[5 5 28 14]); %指定plot输出图片的尺寸，xmin，ymin，width，height
nexttile
yyaxis left;%激活左边的轴
plot(forward(:,1),forward(:,2),'-r',forward(:,1),forward(:,3),'-g');%sim中的x和y
hold on;
plot(ts,screwpos1(:,1),'--c',ts,screwpos1(:,2),'--k');  %采集的电机数据转换为丝杠的位置
ylabel('lead-screw position (m)');
xlabel('time (s)');
ylim([0 0.025]);


yyaxis right;%激活右边的轴
plot(t_input,r1,'-m'); %仿真中的数据（直接读取的丝杠的位置，由于Adams中的模型设置）
hold on
plot(ts,screwpos1(:,3),'--b');  %采集的电机数据转换为丝杠的位置
ylabel('leg plane angle (rad)');
ylim([-0.15 0.15]);
set(gca,'YTick',[-0.1:0.05:0.1]);
ax = gca;
ax.YAxis(1).Exponent=-3;%常数2为指数值，改为0即不使用科学计数法
ax.YAxis(2).Exponent=-3;%常数2为指数值，改为0即不使用科学计数法
set(gca,'FontName','Times new Roman','FontSize',20);

nexttile
yyaxis left;%激活左边的轴
hh1=plot(forward(:,1),forward(:,5),'-r',forward(:,1),forward(:,6),'-g');%sim中的x和y
hold on;
hh2=plot(ts,screwpos2(:,1),'--c',ts,screwpos2(:,2),'--k');  %采集的电机数据转换为丝杠的位置
ylabel('lead-screw position (m)');
xlabel('time (s)');
ylim([0 0.025]);



yyaxis right;%激活右边的轴
hh3=plot(t_input,r2,'-m'); %仿真中的数据（直接读取的丝杠的位置，由于Adams中的模型设置）
hold on
hh4=plot(ts,screwpos2(:,3),'--b');  %采集的电机数据转换为丝杠的位置
ylabel('leg plane angle (rad)');
ylim([-0.15 0.15]);
set(gca,'YTick',[-0.1:0.05:0.1]);
ax = gca;
ax.YAxis(1).Exponent=-3;%常数2为指数值，改为0即不使用科学计数法
ax.YAxis(2).Exponent=-3;%常数2为指数值，改为0即不使用科学计数法
set(gca,'FontName','Times new Roman','FontSize',20);



lgd = legend([hh1;hh3;hh2;hh4],'sim-x','sim-y','sim-r','exp-x','exp-y','exp-r','Orientation','horizontal');
legend('boxoff');
lgd.Layout.Tile = 'south';
set(lgd,'FontSize',28)












%% 计算丝杠位置

function [ans] = calScrewPos(q1,q2,q3)

H0x=0.0152893;
B0y=0.017004;

deltaX=-16*0.0025/26/2/pi*q1;
deltaY=-16*0.0025/26/2/pi*q2;
deltaR = 19/50/28*q3;

By = B0y+deltaY;
Hx = H0x+deltaX;
R = deltaR;
ans = [Hx,By,R];


end




