Gx = 0.0646;
Gy = -0.0763;

Bx = 0.02;
By = -0.0358;

Fx = 0.1036;
Fy = -0.0373;



plot(Gx,Gy,'-*r');
text(Gx+0.03,Gy+0.03,'G');
xlim([])
hold on

plot(Bx,By,'-*r');
text(Bx+0.03,By+0.03,'B');
hold on

plot(Fx,Fy,'-*r');
text(Fx+0.03,Fy+0.03,'F');
hold on

