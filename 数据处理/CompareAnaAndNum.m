clear all
clc
angle1 = readmatrix('invInput.txt');
angle2 = readmatrix('numInput.txt');
t=0.001:0.001:size(angle1(:,1))/1000; %1ms执行一次
t2=0.001:0.001:size(angle2(:,1))/1000; %1ms执行一次
x1=angle1(:,1);
y1=angle1(:,2);
z1=angle1(:,3);
x2=angle1(:,4);
y2=angle1(:,5);
z2=angle1(:,6);
x3=angle1(:,7);
y3=angle1(:,8);
z3=angle1(:,9);
x4=angle1(:,10);
y4=angle1(:,11);
z4=angle1(:,12);
x5=angle1(:,13);
y5=angle1(:,14);
z5=angle1(:,15);
x6=angle1(:,16);
y6=angle1(:,17);
z6=angle1(:,18);


xn1=angle2(:,1);
yn1=angle2(:,2);
zn1=angle2(:,3);
xn2=angle2(:,4);
yn2=angle2(:,5);
zn2=angle2(:,6);
xn3=angle2(:,7);
yn3=angle2(:,8);
zn3=angle2(:,9);
xn4=angle2(:,10);
yn4=angle2(:,11);
zn4=angle2(:,12);
xn5=angle2(:,13);
yn5=angle2(:,14);
zn5=angle2(:,15);
xn6=angle2(:,16);
yn6=angle2(:,17);
zn6=angle2(:,18);


subplot(6,2,1);
plot(t,x1,'r',t,y1,'b',t,z1,'g');
xlabel('time');
ylabel('Ana Solution');
legend('x','y','z');
title('Leg1 Ana Solution');

subplot(6,2,2);
plot(t2,xn1,'r',t2,yn1,'b',t2,zn1,'g');
xlabel('time');
ylabel('Num Solution');
ylim([-0.02 0.02])
legend('x','y','z');
title('Leg1 Num Solution');

subplot(6,2,3);
plot(t,x2,'r',t,y2,'b',t,z2,'g');
xlabel('time');
ylabel('Ana Solution');
legend('x','y','z');
title('Leg2 Ana Solution');

subplot(6,2,4);
plot(t2,xn2,'r',t2,yn2,'b',t2,zn2,'g');
xlabel('time');
ylabel('Num Solution');
legend('x','y','z');
title('Leg2 Num Solution');

subplot(6,2,5);
plot(t,x3,'r',t,y3,'b',t,z3,'g');
xlabel('time');
ylabel('Ana Solution');
legend('x','y','z');
title('Leg3 Ana Solution');

subplot(6,2,6);
plot(t2,xn3,'r',t2,yn3,'b',t2,zn3,'g');
xlabel('time');
ylabel('Num Solution');
legend('x','y','z');
title('Leg3 Num Solution');

subplot(6,2,7);
plot(t,x4,'r',t,y4,'b',t,z4,'g');
xlabel('time');
ylabel('Ana Solution');
legend('x','y','z');
title('Leg4 Ana Solution');

subplot(6,2,8);
plot(t2,xn4,'r',t2,yn4,'b',t2,zn4,'g');
xlabel('time');
ylabel('Num Solution');
legend('x','y','z');
title('Leg4 Num Solution');

subplot(6,2,9);
plot(t,x5,'r',t,y5,'b',t,z5,'g');
xlabel('time');
ylabel('Ana Solution');
legend('x','y','z');
title('Leg5 Ana Solution');

subplot(6,2,10);
plot(t2,xn5,'r',t2,yn5,'b',t2,zn5,'g');
xlabel('time');
ylabel('Num Solution');
legend('x','y','z');
title('Leg5 Num Solution');

subplot(6,2,11);
plot(t,x6,'r',t,y6,'b',t,z6,'g');
xlabel('time');
ylabel('Ana Solution');
legend('x','y','z');
title('Leg6 Ana Solution');

subplot(6,2,12);
plot(t2,xn6,'r',t2,yn6,'b',t2,zn6,'g');
xlabel('time');
ylabel('Num Solution');
legend('x','y','z');
title('Leg6 Num Solution');

suptitle('Comparison of numerical and analytical solutions')



