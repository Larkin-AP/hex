%T型曲线函数图像
%f(a,v)=(a+v^2)/v*a
a=1:0.05:10;
v=1:0.05:10;
[X,Y]=meshgrid(a,v);
Z=(X + Y.^2)/(X.*Y.);
mesh(X,Y,Z);
title('T Curve')

