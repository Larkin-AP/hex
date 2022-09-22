angle = readmatrix('eeTraj.txt');
t=0.001:0.001:size(angle(:,1))/1000; %1ms执行一次
bx=angle(:,4);
by=angle(:,8);
bz=angle(:,12);
x1=angle(:,17);
y1=angle(:,18);
z1=angle(:,19);
x2=angle(:,20);
y2=angle(:,21);
z2=angle(:,22);
x3=angle(:,23);
y3=angle(:,24);
z3=angle(:,25);
x4=angle(:,26);
y4=angle(:,27);
z4=angle(:,28);
x5=angle(:,29);
y5=angle(:,30);
z5=angle(:,31);
x6=angle(:,32);
y6=angle(:,33);
z6=angle(:,34);

subplot(221);
plot(t,bx,'r',t,by,'b',t,bz,'g');
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
suptitle('Tetrapod end-effector trajectory')



