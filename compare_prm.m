%比较不同参数下，leg1的电机输入值
clear all
clc

angle1 = readmatrix('hex_forward(n=4,x=0.35).txt'); %参考值，x=0.3，y-0.4(位置参数) T(1,1) -x=0.1 -y=0.05(行走参数)
% angle2 = readmatrix('hex_forward_change_pos.txt'); %参考值，x=0.4，y-0.4(位置参数) T(1,1) -x=0.1 -y=0.05(行走参数)
% angle3 = readmatrix('hex_forward_change_prm1.txt'); %参考值，x=0.3，y-0.4(位置参数) T(2,2) -x=0.1 -y=0.05(行走参数)
% angle4 = readmatrix('hex_forward_change_prm2.txt'); %参考值，x=0.3，y-0.4(位置参数) T(1,1) -x=0.1 -y=0.02(行走参数)
% angle5 = readmatrix('hex_forward_change_prm3.txt'); %参考值，x=0.3，y-0.4(位置参数) T(1,1) -x=0.05 -y=0.05(行走参数)
% angle6 = readmatrix('hex_forward_change_prm4.txt'); %参考值，x=0.3，y-0.4(位置参数) T(1,1) -x=0.05 -y=0.02(行走参数)

y_val = nan(3,6);


% angle1 = angle1(:,1:3);
% angle2 = angle2(:,1:3);
% angle3 = angle3(:,1:3);
% angle4 = angle4(:,1:3);
% angle5 = angle5(:,1:3);
% angle6 = angle6(:,1:3);

m = max(size(angle1));
t = 0.001:0.001:m/1000;
angle1_1 = angle1(:,1);
angle1_2 = angle1(:,2);
angle1_3 = angle1(:,3);
angle1_4 = angle1(:,4);  %质心x坐标
% angle2_1 = angle2(:,1);
% angle2_2 = angle2(:,2);
% angle2_3 = angle2(:,3);
% 
% angle3_1 = angle3(:,1);
% angle3_2 = angle3(:,2);
% angle3_3 = angle3(:,3);
% 
% angle4_1 = angle4(:,1);
% angle4_2 = angle4(:,2);
% angle4_3 = angle4(:,3);
% 
% angle5_1 = angle5(:,1);
% angle5_2 = angle5(:,2);
% angle5_3 = angle5(:,3);
% 
% angle6_1 = angle6(:,1);
% angle6_2 = angle6(:,2);
% angle6_3 = angle6(:,3);

figure;
plot(t,angle1);
legend('x','y','z');
title('angle1');
hold on

[a1,loc1]=findpeaks(-angle1(:,2));
scatter(loc1/1000,-a1,'black','filled');
% hold on
% [a2,loc2]=findpeaks(-angle2(:,2));
% scatter(loc2/1000,-a2,'black','filled');
% hold on
% [a3,loc3]=findpeaks(-angle3(:,2));
% scatter(loc3/1000,-a3,'black','filled');
% hold on
% [a4,loc4]=findpeaks(-angle4(:,2));
% scatter(loc4/1000,-a4,'black','filled');
% hold on
% [a5,loc5]=findpeaks(-angle5(:,2));
% scatter(loc5/1000,-a5,'black','filled');
% hold on
% [a6,loc6]=findpeaks(-angle6(:,2));
% scatter(loc6/1000,-a6,'black','filled');





y_val(1,1) = min(angle1(:,2));
y_val(2,1) = max(angle1(:,2));

% y_val(1,2) = min(angle2(:,2));
% y_val(2,2) = max(angle2(:,2));
% 
% y_val(1,3) = min(angle3(:,2));
% y_val(2,3) = max(angle3(:,2));
% 
% y_val(1,4) = min(angle4(:,2));
% y_val(2,4) = max(angle4(:,2));
% 
% y_val(1,5) = min(angle5(:,2));
% y_val(2,5) = max(angle5(:,2));
% 
% y_val(1,6) = min(angle6(:,2));
% y_val(2,6) = max(angle6(:,2));

for i=1:6
    y_val(3,i) = -y_val(1,i)/y_val(2,i);
end


%% 画末端轨迹

motor1 = angle1(:,1);
motor2 = angle1(:,2);
motor3 = angle1(:,3);


figure;
plot(t,angle1_4);

ee = nan(m,3);
% figure;
% plot(t,motor1);
% hold on 
% plot(t,motor2);
% 
% col1 = [1 ,1 ,1 ,1]';
% col2 = [2 ,2, 2, 2]';
% 
% yt = ee(loc1,col2);
% xt = ee(loc1,col1);


for i =1:m
    ee(i,:)=Forward_kinematics([motor1(i,1),motor2(i,1),motor3(i,1)]);
end

figure;
plot(t,ee);
legend('x','y','z');
hold on
scatter(loc1/1000,ee(loc1,1),'black','filled');
hold on
scatter(loc1/1000,ee(loc1,2),'black','filled');



