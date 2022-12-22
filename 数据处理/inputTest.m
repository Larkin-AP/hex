angle = readmatrix('inputTraj.txt');
t=0.001:0.001:size(angle(1:end-2,1))/1000; %1ms执行一次
t1=0.001:0.001:size(angle(1:end-3,1))/1000; %1ms执行一次
x1=angle(1:end-2,1);
y1=angle(1:end-2,2);
z1=angle(1:end-2,3);
x2=angle(1:end-2,4);
y2=angle(1:end-2,5);
z2=angle(1:end-2,6);
x3=angle(1:end-2,7);
y3=angle(1:end-2,8);
z3=angle(1:end-2,9);
x4=angle(1:end-2,10);
y4=angle(1:end-2,11);
z4=angle(1:end-2,12);
x5=angle(1:end-2,13);
y5=angle(1:end-2,14);
z5=angle(1:end-2,15);
x6=angle(1:end-2,16);
y6=angle(1:end-2,17);
z6=angle(1:end-2,18);

dx1=diff(x1);
dx2=diff(x2);
dx3=diff(x3);
dx4=diff(x4);
dx5=diff(x5);
dx6=diff(x6);

dy1=diff(y1);
dy2=diff(y2);
dy3=diff(y3);
dy4=diff(y4);
dy5=diff(y5);
dy6=diff(y6);

dz1=diff(z1);
dz2=diff(z2);
dz3=diff(z3);
dz4=diff(z4);
dz5=diff(z5);
dz6=diff(z6);

% subplot(321);
% plot(t,x1,'r')
% title('leg1')
% subplot(322);
% plot(t,x2,'r')
% title('leg2')
% subplot(323);
% plot(t,x3,'r')
% title('leg3')
% subplot(324);
% plot(t,x4,'r')
% title('leg4')
% subplot(325);
% plot(t,x5,'r')
% title('leg5')
% subplot(326);
% plot(t,x6,'r')
% title('leg6')
% suptitle('Motor input in X direction (Tripod)')

% subplot(321);
% plot(t,y1,'r')
% title('leg1')
% subplot(322);
% plot(t,y2,'r')
% title('leg2')
% subplot(323);
% plot(t,y3,'r')
% title('leg3')
% subplot(324);
% plot(t,y4,'r')
% title('leg4')
% subplot(325);
% plot(t,y5,'r')
% title('leg5')
% subplot(326);
% plot(t,y6,'r')
% title('leg6')
% suptitle('Motor input in Y direction (Tripod)')

% subplot(321);
% plot(t,z1,'r')
% title('leg1')
% subplot(322);
% plot(t,z2,'r')
% title('leg2')
% subplot(323);
% plot(t,z3,'r')
% title('leg3')
% subplot(324);
% plot(t,z4,'r')
% title('leg4')
% subplot(325);
% plot(t,z5,'r')
% title('leg5')
% subplot(326);
% plot(t,z6,'r')
% title('leg6')
% suptitle('Motor input in R direction (Tripod)')

subplot(321);
plot(t1,dx1,'r')
title('leg1')
subplot(322);
plot(t1,dx2,'r')
title('leg2')
subplot(323);
plot(t1,dx3,'r')
title('leg3')
subplot(324);
plot(t1,dx4,'r')
title('leg4')
subplot(325);
plot(t1,dx5,'r')
title('leg5')
subplot(326);
plot(t1,dx6,'r')
title('leg6')
suptitle('Motor velocity in X direction (Tripod)')

% subplot(321);
% plot(t1,dy1,'r')
% title('leg1')
% subplot(322);
% plot(t1,dy2,'r')
% title('leg2')
% subplot(323);
% plot(t1,dy3,'r')
% title('leg3')
% subplot(324);
% plot(t1,dy4,'r')
% title('leg4')
% subplot(325);
% plot(t1,dy5,'r')
% title('leg5')
% subplot(326);
% plot(t1,dy6,'r')
% title('leg6')
% suptitle('Motor velocity in Y direction (Tripod)')

% subplot(321);
% plot(t1,dz1,'r')
% title('leg1')
% subplot(322);
% plot(t1,dz2,'r')
% title('leg2')
% subplot(323);
% plot(t1,dz3,'r')
% title('leg3')
% subplot(324);
% plot(t1,dz4,'r')
% title('leg4')
% subplot(325);
% plot(t1,dz5,'r')
% title('leg5')
% subplot(326);
% plot(t1,dz6,'r')
% title('leg6')
% suptitle('Motor velocity in R direction (Tripod)')



