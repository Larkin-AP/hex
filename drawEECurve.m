%根据电机位置，画末端轨迹
clear all
clc

angle = readmatrix('hex_forward(n=4).txt');
motor1 = angle(:,1);
motor2 = angle(:,2);
motor3 = angle(:,3);
m = max(size(motor1));
t=0.001:0.001:m/1000; %1ms执行一次

ee = nan(m,3);
figure;
plot(t,motor1);
hold on 
plot(t,motor2);
loc = [1936,5934];
yt = ee(loc,2);
xt = ee(loc,1);


for i =1:m
    ee(i,:)=Forward_kinematics([motor1(i,1),motor2(i,1),motor3(i,1)]);
end

figure;
plot(t,ee);
legend('x','y','z');
hold on
scatter(loc/1000,ee(loc,1),'black','filled');
hold on
scatter(loc/1000,ee(loc,2),'black','filled');
