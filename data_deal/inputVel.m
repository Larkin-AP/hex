angle = readmatrix('inputVel.txt');
t=0.001:0.001:size(angle(1:end-2,1))/1000; %1ms执行一次
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


subplot(321);
plot(t,dx1,'r')
title('leg1')
subplot(322);
plot(t,x2,'r')
title('leg2')
subplot(323);
plot(t,x3,'r')
title('leg3')
subplot(324);
plot(t,x4,'r')
title('leg4')
subplot(325);
plot(t,x5,'r')
title('leg5')
subplot(326);
plot(t,x6,'r')
title('leg6')
suptitle('Motor input in X direction (Tripod)')

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




