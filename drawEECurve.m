%根据电机位置，画末端轨迹
clear all
clc

angle = readmatrix('hex_forward.txt');
motor1 = angle(:,1);
motor2 = angle(:,2);
motor3 = angle(:,3);
m = max(size(motor1));
t=0.001:0.001:m/1000; %1ms执行一次

ee = nan(m,3);

for i =1:m
    ee(i,:)=Forward_kinematics([motor1(i,1),motor2(i,1),motor3(i,1)]);
end

plot(t,ee);
legend('x','y','z');
