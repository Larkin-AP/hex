%画ee，身体末端和足端轨迹ee[34]
clear all
clc

angle = readmatrix('movebody.txt');
% motor = readmatrix('MotorPos.txt');

body_x = angle(:,4);
body_y = angle(:,8);
body_z = angle(:,12);
plot(body_x);
hold on
plot(body_y);
hold on 
plot(body_z);
legend('x','y','z');

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

% m1 = motor(:,1);
% m2 = motor(:,2);
% m3 = motor(:,3);
% 
% m4 = motor(:,4);
% m5 = motor(:,5);
% m6 = motor(:,6);
% 
% m7 = motor(:,7);
% m8 = motor(:,8);
% m9 = motor(:,9);
% 
% m10 = motor(:,10);
% m11 = motor(:,11);
% m12 = motor(:,12);
% 
% m13 = motor(:,13);
% m14 = motor(:,14);
% m15 = motor(:,15);
% 
% m16 = motor(:,16);
% m17 = motor(:,17);
% m18 = motor(:,18);

% figure;
% plot(leg1_x);
% hold on
% plot(leg1_y);
% hold on
% plot(leg1_z);
% hold on
% plot(leg3_x);
% hold on
% plot(leg3_y);
% hold on
% plot(leg3_z);
% hold on
% title('leg2')
% 
% figure;
% plot(leg3_x);
% hold on
% plot(leg3_y);
% hold on
% plot(leg3_z);
% hold on
% legend('x','y','z');
% title('leg3')
% 
% figure;
% plot(m4);
% hold on
% plot(m5);
% hold on
% plot(m6);
% hold on
% plot(m7);
% hold on
% plot(m8);
% hold on
% plot(m9);
% hold on
% legend('2x','2y','2z','3x','3y','3z');
scatter3(leg1_x',leg1_z',leg1_y');
hold on
scatter3(leg2_x',leg2_z',leg2_y');
hold on
scatter3(leg3_x',leg3_z',leg3_y');
hold on
scatter3(leg4_x',leg4_z',leg4_y');
hold on
scatter3(leg5_x',leg5_z',leg5_y');
hold on
scatter3(leg6_x',leg6_z',leg6_y');
hold on
xlabel('x');
ylabel('z');
zlabel('y');
set(gca,'YDir','reverse');        %将x轴方向设置为反向(从右到左递增)。
axis equal
view(40,35)

% figure;
% plot(leg2_x);
% hold on
% plot(leg5_x);
% hold on
% 
% figure;
% plot(leg3_x);
% hold on
% plot(leg6_x);
% hold on
% 
% figure;
% plot(leg1_y);
% hold on
% plot(leg4_y);
% hold on
% 
% figure;
% plot(leg2_y);
% hold on
% plot(leg5_y);
% hold on
% 
% figure;
% plot(leg3_y);
% hold on
% plot(leg6_y);
% hold on
% legend('1','2','3','4','5','6');
% plot(leg1_y);
% hold on

plot(body_x);
hold on
plot(body_y);
hold on
plot(body_z);
hold on

