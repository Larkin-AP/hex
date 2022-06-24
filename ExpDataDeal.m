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

angle2_1 = angle2(:,1);
angle2_2 = angle2(:,2);
angle2_3 = angle2(:,3);

angle3_1 = angle3(:,1);
angle3_2 = angle3(:,2);
angle3_3 = angle3(:,3);

angle1_4 = angle1(:,4);
angle1_5 = angle1(:,5);
angle1_6 = angle1(:,6);

angle2_4 = angle2(:,4);
angle2_5 = angle2(:,5);
angle2_6 = angle2(:,6);

angle3_4 = angle3(:,4);
angle3_5 = angle3(:,5);
angle3_6 = angle3(:,6);

%一次差分
d_angle1_1=(angle1_1(2:end)-angle1_1(1:end-1))*1000;
d_angle1_2=(angle1_2(2:end)-angle1_2(1:end-1))*1000;
d_angle1_3=(angle1_3(2:end)-angle1_3(1:end-1))*1000;

d_angle2_1=(angle2_1(2:end)-angle2_1(1:end-1))*1000;
d_angle2_2=(angle2_2(2:end)-angle2_2(1:end-1))*1000;
d_angle2_3=(angle2_3(2:end)-angle2_3(1:end-1))*1000;

d_angle3_1=(angle3_1(2:end)-angle3_1(1:end-1))*1000;
d_angle3_2=(angle3_2(2:end)-angle3_2(1:end-1))*1000;
d_angle3_3=(angle3_3(2:end)-angle3_3(1:end-1))*1000;

d_angle1_4=(angle1_4(2:end)-angle1_4(1:end-1))*1000;
d_angle1_5=(angle1_5(2:end)-angle1_5(1:end-1))*1000;
d_angle1_6=(angle1_6(2:end)-angle1_6(1:end-1))*1000;

d_angle2_4=(angle2_4(2:end)-angle2_4(1:end-1))*1000;
d_angle2_5=(angle2_5(2:end)-angle2_5(1:end-1))*1000;
d_angle2_6=(angle2_6(2:end)-angle2_6(1:end-1))*1000;

d_angle3_4=(angle3_4(2:end)-angle3_4(1:end-1))*1000;
d_angle3_5=(angle3_5(2:end)-angle3_5(1:end-1))*1000;
d_angle3_6=(angle3_6(2:end)-angle3_6(1:end-1))*1000;

%二次差分
dd_angle1_1=(d_angle1_1(2:end)-d_angle1_1(1:end-1))*1000;
dd_angle1_2=(d_angle1_2(2:end)-d_angle1_2(1:end-1))*1000;
dd_angle1_3=(d_angle1_3(2:end)-d_angle1_3(1:end-1))*1000;

dd_angle2_1=(d_angle2_1(2:end)-d_angle2_1(1:end-1))*1000;
dd_angle2_2=(d_angle2_2(2:end)-d_angle2_2(1:end-1))*1000;
dd_angle2_3=(d_angle2_3(2:end)-d_angle2_3(1:end-1))*1000;

dd_angle3_1=(d_angle3_1(2:end)-d_angle3_1(1:end-1))*1000;
dd_angle3_2=(d_angle3_2(2:end)-d_angle3_2(1:end-1))*1000;
dd_angle3_3=(d_angle3_3(2:end)-d_angle3_3(1:end-1))*1000;

dd_angle1_4=(d_angle1_4(2:end)-d_angle1_4(1:end-1))*1000;
dd_angle1_5=(d_angle1_5(2:end)-d_angle1_5(1:end-1))*1000;
dd_angle1_6=(d_angle1_6(2:end)-d_angle1_6(1:end-1))*1000;

dd_angle2_4=(d_angle2_4(2:end)-d_angle2_4(1:end-1))*1000;
dd_angle2_5=(d_angle2_5(2:end)-d_angle2_5(1:end-1))*1000;
dd_angle2_6=(d_angle2_6(2:end)-d_angle2_6(1:end-1))*1000;

dd_angle3_4=(d_angle3_4(2:end)-d_angle3_4(1:end-1))*1000;
dd_angle3_5=(d_angle3_5(2:end)-d_angle3_5(1:end-1))*1000;
dd_angle3_6=(d_angle3_6(2:end)-d_angle3_6(1:end-1))*1000;

%% 画图 前进方向

%位置图
h1 = figure;
figure(h1)

subplot(3,2,1);
plot(t,angle1_1,'r',t,angle1_2,'g',t,angle1_3,'b');
legend('x','y','z');
xlabel('t(s)');
ylabel('q(rad)');
title('leg1 motor pos');

subplot(3,2,2);
plot(t,angle1_4,'r',t,angle1_5,'g',t,angle1_6,'b');
legend('x','y','z');
xlabel('t(s)');
ylabel('q(rad)');
title('leg2 motor pos');

subplot(3,2,3);
plot(t1,d_angle1_1,'r',t1,d_angle1_2,'g',t1,d_angle1_3,'b');
legend('x','y','z');
xlabel('t(s)');
ylabel('dq(rad/s)');
title('leg1 motor vel');

subplot(3,2,4);
plot(t1,d_angle1_4,'r',t1,d_angle1_5,'g',t1,d_angle1_6,'b');
legend('x','y','z');
xlabel('t(s)');
ylabel('dq(rad/s)');
title('leg2 motor vel');

subplot(3,2,5);
plot(t2,dd_angle1_1,'r',t2,dd_angle1_2,'g',t2,dd_angle1_3,'b');
legend('x','y','z');
xlabel('t(s)');
ylabel('ddq(rad/s\^2)');
title('leg1 motor acc');

subplot(3,2,6);
plot(t2,dd_angle1_4,'r',t2,dd_angle1_5,'g',t2,dd_angle1_6,'b');
legend('x','y','z');
xlabel('t(s)');
ylabel('ddq(rad/s\^2)');
title('leg2 motor acc');


%% 画图 侧移方向

%位置图
h2 = figure;
figure(h2)

subplot(3,2,1);
plot(t,angle2_1,'r',t,angle2_2,'g',t,angle2_3,'b');
legend('x','y','z');
xlabel('t(s)');
ylabel('q(rad)');
title('leg1 motor pos');

subplot(3,2,2);
plot(t,angle2_4,'r',t,angle2_5,'g',t,angle2_6,'b');
legend('x','y','z');
xlabel('t(s)');
ylabel('q(rad)');
title('leg2 motor pos');

subplot(3,2,3);
plot(t1,d_angle2_1,'r',t1,d_angle2_2,'g',t1,d_angle2_3,'b');
legend('x','y','z');
xlabel('t(s)');
ylabel('dq(rad/s)');
title('leg1 motor vel');

subplot(3,2,4);
plot(t1,d_angle2_4,'r',t1,d_angle2_5,'g',t1,d_angle2_6,'b');
legend('x','y','z');
xlabel('t(s)');
ylabel('dq(rad/s)');
title('leg2 motor vel');

subplot(3,2,5);
plot(t2,dd_angle2_1,'r',t2,dd_angle2_2,'g',t2,dd_angle2_3,'b');
legend('x','y','z');
xlabel('t(s)');
ylabel('ddq(rad/s\^2)');
title('leg1 motor acc');

subplot(3,2,6);
plot(t2,dd_angle2_4,'r',t2,dd_angle2_5,'g',t2,dd_angle2_6,'b');
legend('x','y','z');
xlabel('t(s)');
ylabel('ddq(rad/s\^2)');
title('leg2 motor acc');


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

h3=figure;
figure(h3);
plot(forward(:,1),forward(:,2),forward(:,1),forward(:,3),t_input,r1);
hold on
plot(ts,screwpos1(:,1),'--r',ts,screwpos1(:,2),'--g',ts,screwpos1(:,3),'b');
legend('sx','sy','sz','px','py','pz')

h4=figure;
figure(h4);
plot(forward(:,1),forward(:,5),forward(:,1),forward(:,6),t_input,r2)
hold on
plot(ts,screwpos2(:,1),'--r',ts,screwpos2(:,2),'--g',ts,screwpos2(:,3),'b');
legend('sx','sy','sz','px','py','pz')





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




