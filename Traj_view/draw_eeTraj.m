clear all
clc

%读取数据
angle = readmatrix('eeTraj.txt');

%画身体轨迹
body_x = angle(:,4);
body_y = angle(:,8);
body_z = angle(:,12);
h1=figure;
figure(h1);

plot(body_x);
hold on
plot(body_y);
hold on 
plot(body_z);
hold off
legend('x','y','z');

%定义身体轨迹
leg1_x = angle(:,17);
leg1_y = angle(:,18);
leg1_z = angle(:,19);

leg2_x = angle(:,20);
leg2_y = angle(:,21);
leg2_z = angle(:,22);

leg3_x = angle(:,23);
leg3_y = angle(:,24);
leg3_z = angle(:,25);

leg4_x = angle(:,26);
leg4_y = angle(:,27);
leg4_z = angle(:,28);

leg5_x = angle(:,29);
leg5_y = angle(:,30);
leg5_z = angle(:,31);

leg6_x = angle(:,32);
leg6_y = angle(:,33);
leg6_z = angle(:,34);

h2=figure;
figure(h2);
plot(leg1_x);
hold on
plot(leg1_y);
hold on
plot(leg1_z);