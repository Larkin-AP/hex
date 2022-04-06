%连续性检测，使用时把文件拷贝至log文件夹
%% 看末端位置是否连续
%获取数据
clear all
clc
 
%产生变量
ee_pos_in_world = readmatrix('leg1EndTraj1.txt');  %单位为m
mot_input = readmatrix('leg1MotorPos1.txt');  %单位为rad
t=0.001:0.001:size(ee_pos_in_world(:,1))/1000; %1ms执行一次


body_x=ee_pos_in_world(:,4);
body_y=ee_pos_in_world(:,8);
body_z=ee_pos_in_world(:,12);


x1=ee_pos_in_world(:,17);
y1=ee_pos_in_world(:,18);
z1=ee_pos_in_world(:,19);
x2=ee_pos_in_world(:,20);
y2=ee_pos_in_world(:,21);
z2=ee_pos_in_world(:,22);
x3=ee_pos_in_world(:,23);
y3=ee_pos_in_world(:,24);
z3=ee_pos_in_world(:,25);
x4=ee_pos_in_world(:,26);
y4=ee_pos_in_world(:,27);
z4=ee_pos_in_world(:,28);
x5=ee_pos_in_world(:,29);
y5=ee_pos_in_world(:,30);
z5=ee_pos_in_world(:,31);
x6=ee_pos_in_world(:,32);
y6=ee_pos_in_world(:,33);
z6=ee_pos_in_world(:,34);

x1_rel=x1-body_x;
x2_rel=x2-body_x;
x3_rel=x3-body_x;
x4_rel=x4-body_x;
x5_rel=x5-body_x;
x6_rel=x6-body_x;

y1_rel=y1-body_y;
y2_rel=y2-body_y;
y3_rel=y3-body_y;
y4_rel=y4-body_y;
y5_rel=y5-body_y;
y6_rel=y6-body_y;

z1_rel=z1-body_z;
z2_rel=z2-body_z;
z3_rel=z3-body_z;
z4_rel=z4-body_z;
z5_rel=z5-body_z;
z6_rel=z6-body_z;

dif_x1=x1_rel(2:end)-x1_rel(1:end-1);
dif_x2=x2_rel(2:end)-x2_rel(1:end-1);
dif_x3=x3_rel(2:end)-x3_rel(1:end-1);
dif_x4=x4_rel(2:end)-x4_rel(1:end-1);
dif_x5=x5_rel(2:end)-x5_rel(1:end-1);
dif_x6=x6_rel(2:end)-x6_rel(1:end-1);

dif_x1=dif_x1*1000; %单位 m/s
dif_x2=dif_x2*1000;
dif_x3=dif_x3*1000;
dif_x4=dif_x4*1000;
dif_x5=dif_x5*1000;
dif_x6=dif_x6*1000;

dif_y1=y1_rel(2:end)-y1_rel(1:end-1);
dif_y2=y2_rel(2:end)-y2_rel(1:end-1);
dif_y3=y3_rel(2:end)-y3_rel(1:end-1);
dif_y4=y4_rel(2:end)-y4_rel(1:end-1);
dif_y5=y5_rel(2:end)-y5_rel(1:end-1);
dif_y6=y6_rel(2:end)-y6_rel(1:end-1);

dif_y1=dif_y1*1000;
dif_y2=dif_y2*1000;
dif_y3=dif_y3*1000;
dif_y4=dif_y4*1000;
dif_y5=dif_y5*1000;
dif_y6=dif_y6*1000;

dif_z1=z1_rel(2:end)-z1_rel(1:end-1);
dif_z2=z2_rel(2:end)-z2_rel(1:end-1);
dif_z3=z3_rel(2:end)-z3_rel(1:end-1);
dif_z4=z4_rel(2:end)-z4_rel(1:end-1);
dif_z5=z5_rel(2:end)-z5_rel(1:end-1);
dif_z6=z6_rel(2:end)-z6_rel(1:end-1);

dif_z1=dif_z1*1000;
dif_z2=dif_z2*1000;
dif_z3=dif_z3*1000;
dif_z4=dif_z4*1000;
dif_z5=dif_z5*1000;
dif_z6=dif_z6*1000;

t1=t(1:end-1);

ddif_x1=dif_x1(2:end)-dif_x1(1:end-1);  %单位m/s*s
ddif_x2=dif_x2(2:end)-dif_x2(1:end-1);
ddif_x3=dif_x3(2:end)-dif_x3(1:end-1);
ddif_x4=dif_x4(2:end)-dif_x4(1:end-1);
ddif_x5=dif_x5(2:end)-dif_x5(1:end-1);
ddif_x6=dif_x6(2:end)-dif_x6(1:end-1);

ddif_y1=dif_y1(2:end)-dif_y1(1:end-1);
ddif_y2=dif_y2(2:end)-dif_y2(1:end-1);
ddif_y3=dif_y3(2:end)-dif_y3(1:end-1);
ddif_y4=dif_y4(2:end)-dif_y4(1:end-1);
ddif_y5=dif_y5(2:end)-dif_y5(1:end-1);
ddif_y6=dif_y6(2:end)-dif_y6(1:end-1);

ddif_z1=dif_z1(2:end)-dif_z1(1:end-1);
ddif_z2=dif_z2(2:end)-dif_z2(1:end-1);
ddif_z3=dif_z3(2:end)-dif_z3(1:end-1);
ddif_z4=dif_z4(2:end)-dif_z4(1:end-1);
ddif_z5=dif_z5(2:end)-dif_z5(1:end-1);
ddif_z6=dif_z6(2:end)-dif_z6(1:end-1);

t2=t1(1:end-1);

%电机量 18个电机的位置变化
m1=mot_input(:,1);  %单位rad
m2=mot_input(:,2);
m3=mot_input(:,3);
m4=mot_input(:,4);
m5=mot_input(:,5);
m6=mot_input(:,6);
m7=mot_input(:,7);
m8=mot_input(:,8);
m9=mot_input(:,9);
m10=mot_input(:,10);
m11=mot_input(:,11);
m12=mot_input(:,12);
m13=mot_input(:,13);
m14=mot_input(:,14);
m15=mot_input(:,15);
m16=mot_input(:,16);
m17=mot_input(:,17);
m18=mot_input(:,18);

%一次差分
dif_m1=m1(2:end)-m1(1:end-1);  %rad/ms
dif_m2=m2(2:end)-m2(1:end-1);
dif_m3=m3(2:end)-m3(1:end-1);
dif_m4=m4(2:end)-m4(1:end-1);
dif_m5=m5(2:end)-m5(1:end-1);
dif_m6=m6(2:end)-m6(1:end-1);
dif_m7=m7(2:end)-m7(1:end-1);
dif_m8=m8(2:end)-m8(1:end-1);
dif_m9=m9(2:end)-m9(1:end-1);
dif_m10=m10(2:end)-m10(1:end-1);
dif_m11=m11(2:end)-m11(1:end-1);
dif_m12=m12(2:end)-m12(1:end-1);
dif_m13=m13(2:end)-m13(1:end-1);
dif_m14=m14(2:end)-m14(1:end-1);
dif_m15=m15(2:end)-m15(1:end-1);
dif_m16=m16(2:end)-m16(1:end-1);
dif_m17=m17(2:end)-m17(1:end-1);
dif_m18=m18(2:end)-m18(1:end-1);

dif_m1=dif_m1*1000; %rad/s
dif_m2=dif_m2*1000;
dif_m3=dif_m3*1000;
dif_m4=dif_m4*1000;
dif_m5=dif_m5*1000;
dif_m6=dif_m6*1000;
dif_m7=dif_m7*1000;
dif_m8=dif_m8*1000;
dif_m9=dif_m9*1000;
dif_m10=dif_m10*1000;
dif_m11=dif_m11*1000;
dif_m12=dif_m12*1000;
dif_m13=dif_m13*1000;
dif_m14=dif_m14*1000;
dif_m15=dif_m15*1000;
dif_m16=dif_m16*1000;
dif_m17=dif_m17*1000;
dif_m18=dif_m18*1000;

%二次差分
ddif_m1=dif_m1(2:end)-dif_m1(1:end-1);  %rad/s/s
ddif_m2=dif_m2(2:end)-dif_m2(1:end-1);
ddif_m3=dif_m3(2:end)-dif_m3(1:end-1);
ddif_m4=dif_m4(2:end)-dif_m4(1:end-1);
ddif_m5=dif_m5(2:end)-dif_m5(1:end-1);
ddif_m6=dif_m6(2:end)-dif_m6(1:end-1);
ddif_m7=dif_m7(2:end)-dif_m7(1:end-1);
ddif_m8=dif_m8(2:end)-dif_m8(1:end-1);
ddif_m9=dif_m9(2:end)-dif_m9(1:end-1);
ddif_m10=dif_m10(2:end)-dif_m10(1:end-1);
ddif_m11=dif_m11(2:end)-dif_m11(1:end-1);
ddif_m12=dif_m12(2:end)-dif_m12(1:end-1);
ddif_m13=dif_m13(2:end)-dif_m13(1:end-1);
ddif_m14=dif_m14(2:end)-dif_m14(1:end-1);
ddif_m15=dif_m15(2:end)-dif_m15(1:end-1);
ddif_m16=dif_m16(2:end)-dif_m16(1:end-1);
ddif_m17=dif_m17(2:end)-dif_m17(1:end-1);
ddif_m18=dif_m18(2:end)-dif_m18(1:end-1);

%% 身体和三条腿的关系
figure(1);
subplot(221);
plot(t,body_x,'r',t,body_y,'b',t,body_z,'--g');
xlabel('time');
ylabel('Position');
legend('x','y','z');
title('Body coordinate trajectory');

subplot(222);
plot(t,x1,'r',t,x2,'b',t,x3,'g');
xlabel('time');
ylabel('Position');
legend('Leg1','Leg2','Leg3');
title('Leg1,leg2 and leg3 x coordinate trajectory')

subplot(223);
plot(t,y1,'r',t,y2,'b',t,y3,'g');
xlabel('time');
ylabel('Position');
legend('Leg1','Leg2','Leg3');
title('Leg1,leg2 and leg3 y coordinate trajectory')

subplot(224);
plot(t,z1,'r',t,z2,'b',t,z3,'g');
xlabel('time');
ylabel('Position');
legend('Leg1','Leg2','Leg3');
title('Leg1,leg2 and leg3 z coordinate trajectory')
suptitle('Tripod end-effector trajectory')

%% 六条腿在世界坐标系下的坐标

figure(2);
subplot(321);
plot(t,x1,'r',t,y1,'b',t,z1,'--g');
xlabel('time');
ylabel('Position');
legend({'x1','y1','z1'},'FontSize',12);
title('Leg1 x,y,z coordinate trajectory')

subplot(322);
plot(t,x2,'r',t,y2,'b',t,z2,'--g');
xlabel('time');
ylabel('Position');
legend({'x2','y2','z2'},'FontSize',12);
title('Leg2 x,y,z coordinate trajectory')

subplot(323);
plot(t,x3,'r',t,y3,'b',t,z3,'--g');
xlabel('time');
ylabel('Position');
legend({'x3','y3','z3'},'FontSize',12);
title('Leg3 x,y,z coordinate trajectory')

subplot(324);
plot(t,x4,'r',t,y4,'b',t,z4,'--g');
xlabel('time');
ylabel('Position');
legend({'x4','y4','z4'},'FontSize',12);
title('Leg4 x,y,z coordinate trajectory')

subplot(325);
plot(t,x5,'r',t,y5,'b',t,z5,'--g');
xlabel('time');
ylabel('Position');
legend({'x5','y5','z5'},'FontSize',12);
title('Leg5 x,y,z coordinate trajectory')

subplot(326);
plot(t,x6,'r',t,y6,'b',t,z6,'--g');
xlabel('time');
ylabel('Position');
legend({'x6','y6','z6'},'FontSize',12);
title('Leg6 x,y,z coordinate trajectory')
suptitle('六条腿在世界坐标系下的坐标表示')

%% 腿相当于身体的规划，腿坐标减去身体坐标

figure(3);
subplot(321);
plot(t,x1_rel,'r',t,y1_rel,'b',t,z1_rel,'--g');
xlabel('time');
ylabel('Position');
legend({'x1\_rel','y1\_rel','z1\_rel'},'FontSize',12);
title('Leg1 x,y,z relative coordinate trajectory')

subplot(322);
plot(t,x2_rel,'r',t,y2_rel,'b',t,z2_rel,'--g');
xlabel('time');
ylabel('Position');
legend({'x2\_rel','y2\_rel','z2\_rel'},'FontSize',12);
title('Leg2 x,y,z relative coordinate trajectory')

subplot(323);
plot(t,x3_rel,'r',t,y3_rel,'b',t,z3_rel,'--g');
xlabel('time');
ylabel('Position');
legend({'x3\_rel','y3\_rel','z3\_rel'},'FontSize',12);
title('Leg3 x,y,z relative coordinate trajectory')

subplot(324);
plot(t,x4_rel,'r',t,y4_rel,'b',t,z4_rel,'--g');
xlabel('time');
ylabel('Position');
legend({'x4\_rel','y4\_rel','z4\_rel'},'FontSize',12);
title('Leg4 x,y,z relative coordinate trajectory')

subplot(325);
plot(t,x5_rel,'r',t,y5_rel,'b',t,z5_rel,'--g');
xlabel('time');
ylabel('Position');
legend({'x5\_rel','y5\_rel','z5\_rel'},'FontSize',12);
title('Leg5 x,y,z relative coordinate trajectory')

subplot(326);
plot(t,x6_rel,'r',t,y6_rel,'b',t,z6_rel,'--g');
xlabel('time');
ylabel('Position');
legend({'x6\_rel','y6\_rel','z6\_rel'},'FontSize',12);
title('Leg6 x,y,z relative coordinate trajectory')
suptitle('六条腿相对于身体的坐标')

%% 速度 离散点差分 dif_x = x_rel_n+1-x_rel_n 直接用相对坐标

figure(4);
subplot(321);
plot(t1,dif_x1,'r',t1,dif_y1,'b',t1,dif_z1,'--g');
xlabel('time');
ylabel('Velocity');
legend({'dif\_x1','dif\_y1','dif\_z1'},'FontSize',12);
title('Leg1 x,y,z difference coordinate trajectory')

subplot(322);
plot(t1,dif_x2,'r',t1,dif_y2,'b',t1,dif_z2,'--g');
xlabel('time');
ylabel('Velocity');
legend({'dif\_x2','dif\_y2','dif\_z2'},'FontSize',12);
title('Leg2 x,y,z difference coordinate trajectory')

subplot(323);
plot(t1,dif_x3,'r',t1,dif_y3,'b',t1,dif_z3,'--g');
xlabel('time');
ylabel('Velocity');
legend({'dif\_x3','dif\_y3','dif\_z3'},'FontSize',12);
title('Leg3 x,y,z difference coordinate trajectory')

subplot(324);
plot(t1,dif_x4,'r',t1,dif_y4,'b',t1,dif_z4,'--g');
xlabel('time');
ylabel('Velocity');
legend({'dif\_x4','dif\_y4','dif\_z4'},'FontSize',12);
title('Leg4 x,y,z difference coordinate trajectory')

subplot(325);
plot(t1,dif_x5,'r',t1,dif_y5,'b',t1,dif_z5,'--g');
xlabel('time');
ylabel('Velocity');
legend({'dif\_x5','dif\_y5','dif\_z5'},'FontSize',12);
title('Leg5 x,y,z difference coordinate trajectory')

subplot(326);
plot(t1,dif_x6,'r',t1,dif_y6,'b',t1,dif_z6,'--g');
xlabel('time');
ylabel('Velocity');
legend({'dif\_x6','dif\_y6','dif\_z6'},'FontSize',12);
title('Leg6 x,y,z difference coordinate trajectory')
suptitle('六条腿相对于身体的速度')

%% 加速度 ddif_x=dif_x_n+1-dif_x_n 离散点二次差分，还是相对坐标系下

figure(5);
subplot(321);
plot(t2,ddif_x1,'r',t2,ddif_y1,'b',t2,ddif_z1,'--g');
xlabel('time');
ylabel('Acceleration');
legend({'ddif\_x1','ddif\_y1','ddif\_z1'},'FontSize',12);
title('Leg1 x,y,z second order difference coordinate trajectory')

subplot(322);
plot(t2,ddif_x2,'r',t2,ddif_y2,'b',t2,ddif_z2,'--g');
xlabel('time');
ylabel('Acceleration');
legend({'ddif\_x2','ddif\_y2','ddif\_z2'},'FontSize',12);
title('Leg2 x,y,z second order difference coordinate trajectory')

subplot(323);
plot(t2,ddif_x3,'r',t2,ddif_y3,'b',t2,ddif_z3,'--g');
xlabel('time');
ylabel('Acceleration');
legend({'ddif\_x3','ddif\_y3','ddif\_z3'},'FontSize',12);
title('Leg3 x,y,z second order difference coordinate trajectory')

subplot(324);
plot(t2,ddif_x4,'r',t2,ddif_y4,'b',t2,ddif_z4,'--g');
xlabel('time');
ylabel('Acceleration');
legend({'ddif\_x4','ddif\_y4','ddif\_z4'},'FontSize',12);
title('Leg4 x,y,z second order difference coordinate trajectory')

subplot(325);
plot(t2,ddif_x5,'r',t2,ddif_y5,'b',t2,ddif_z5,'--g');
xlabel('time');
ylabel('Acceleration');
legend({'ddif\_x5','ddif\_y5','ddif\_z5'},'FontSize',12);
title('Leg5 x,y,z second order difference coordinate trajectory')

subplot(326);
plot(t2,ddif_x6,'r',t2,ddif_y6,'b',t2,ddif_z6,'--g');
xlabel('time');
ylabel('Acceleration');
legend({'ddif\_x6','ddif\_y6','ddif\_z6'},'FontSize',12);
title('Leg6 x,y,z second order difference coordinate trajectory')
suptitle('六条腿相对于身体的加速度')

%% 18个电机的位置变化

figure(6);
subplot(321);
plot(t,m1,'r',t,m2,'b',t,m3,'--g');
xlabel('time');
ylabel('Position');
legend({'m1','m2','m3'},'FontSize',12);
title('Leg1 three motors position trajectory')

subplot(322);
plot(t,m4,'r',t,m5,'b',t,m6,'--g');
xlabel('time');
ylabel('Position');
legend({'m4','m5','m6'},'FontSize',12);
title('Leg2 three motors position trajectory')

subplot(323);
plot(t,m7,'r',t,m8,'b',t,m9,'--g');
xlabel('time');
ylabel('Position');
legend({'m7','m8','m9'},'FontSize',12);
title('Leg3 three motors position trajectory')

subplot(324);
plot(t,m10,'r',t,m11,'b',t,m12,'--g');
xlabel('time');
ylabel('Position');
legend({'m10','m11','m12'},'FontSize',12);
title('Leg4 three motors position trajectory')

subplot(325);
plot(t,m13,'r',t,m14,'b',t,m15,'--g');
xlabel('time');
ylabel('Position');
legend({'m13','m14','m15'},'FontSize',12);
title('Leg5 three motors position trajectory')

subplot(326);
plot(t,m16,'r',t,m17,'b',t,m18,'--g');
xlabel('time');
ylabel('Position');
legend({'m16','m17','m18'},'FontSize',12);
title('Leg6 three motors position trajectory')
suptitle('18个电机的位置变化')

%% 18个电机的一次差值

figure(7);
subplot(321);
plot(t1,dif_m1,'r',t1,dif_m2,'b',t1,dif_m3,'--g');
xlabel('time');
ylabel('Velocity');
legend({'dif\_m1','dif\_m2','dif\_m3'},'FontSize',12);
title('Leg1 three motors velocity trajectory')

subplot(322);
plot(t1,dif_m4,'r',t1,dif_m5,'b',t1,dif_m6,'--g');
xlabel('time');
ylabel('Velocity');
legend({'dif\_m4','dif\_m5','dif\_m6'},'FontSize',12);
title('Leg2 three motors velocity trajectory')

subplot(323);
plot(t1,dif_m7,'r',t1,dif_m8,'b',t1,dif_m9,'--g');
xlabel('time');
ylabel('Velocity');
legend({'dif\_m7','dif\_m8','dif\_m9'},'FontSize',12);
title('Leg3 three motors velocity trajectory')

subplot(324);
plot(t1,dif_m10,'r',t1,dif_m11,'b',t1,dif_m12,'--g');
xlabel('time');
ylabel('Velocity');
legend({'dif\_m10','dif\_m11','dif\_m12'},'FontSize',12);
title('Leg4 three motors velocity trajectory')

subplot(325);
plot(t1,dif_m13,'r',t1,dif_m14,'b',t1,dif_m15,'--g');
xlabel('time');
ylabel('Velocity');
legend({'dif\_m13','dif\_m14','dif\_m15'},'FontSize',12);
title('Leg5 three motors velocity trajectory')

subplot(326);
plot(t1,dif_m16,'r',t1,dif_m17,'b',t1,dif_m18,'--g');
xlabel('time');
ylabel('Velocity');
legend({'dif\_m16','dif\_m17','dif\_m18'},'FontSize',12);
title('Leg6 three motors position trajectory')
suptitle('18个电机的速度变化')

%% 18个电机的二次差分

figure(8);
subplot(321);
plot(t2,ddif_m1,'r',t2,ddif_m2,'b',t2,ddif_m3,'--g');
xlabel('time');
ylabel('Acceleration');
legend({'ddif\_m1','ddif\_m2','ddif\_m3'},'FontSize',12);
title('Leg1 three motors acceleration trajectory')

subplot(322);
plot(t2,ddif_m4,'r',t2,ddif_m5,'b',t2,ddif_m6,'--g');
xlabel('time');
ylabel('Acceleration');
legend({'ddif\_m4','ddif\_m5','ddif\_m6'},'FontSize',12);
title('Leg2 three motors acceleration trajectory')

subplot(323);
plot(t2,ddif_m7,'r',t2,ddif_m8,'b',t2,ddif_m9,'--g');
xlabel('time');
ylabel('Acceleration');
legend({'ddif\_m7','ddif\_m8','ddif\_m9'},'FontSize',12);
title('Leg3 three motors acceleration trajectory')

subplot(324);
plot(t2,ddif_m10,'r',t2,ddif_m11,'b',t2,ddif_m12,'--g');
xlabel('time');
ylabel('Acceleration');
legend({'ddif\_m10','ddif\_m11','ddif\_m12'},'FontSize',12);
title('Leg4 three motors acceleration trajectory')

subplot(325);
plot(t2,ddif_m13,'r',t2,ddif_m14,'b',t2,ddif_m15,'--g');
xlabel('time');
ylabel('Acceleration');
legend({'ddif\_m13','ddif\_m14','ddif\_m15'},'FontSize',12);
title('Leg5 three motors acceleration trajectory')

subplot(326);
plot(t2,ddif_m16,'r',t2,ddif_m17,'b',t2,ddif_m18,'--g');
xlabel('time');
ylabel('Acceleration');
legend({'ddif\_m16','ddif\_m17','ddif\_m18'},'FontSize',12);
title('Leg6 three motors acceleration trajectory')
suptitle('18个电机的加速度变化')





