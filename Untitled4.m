Gx = 0.0624;
Gy = -0.0781;

Bx = 0.02;
By = -0.0357;

Fx = 0.1048;
Fy = -0.1206;


h1=figure;
figure(h1)
plot(Gx,Gy,'-*r');
text(Gx+0.01,Gy+0.01,'G');
xlim([0,0.6])
ylim([-0.6,0.1])

hold on

plot(Bx,By,'-*r');
text(Bx+0.01,By+0.01,'B');
hold on

plot(Fx,Fy,'-*r');
text(Fx+0.01,Fy+0.01,'F');
hold on



