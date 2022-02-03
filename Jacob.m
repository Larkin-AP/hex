function [Jac]=Jacob(mot_pos)
%计算雅可比矩阵，腿坐标系下（3ｄ）
%dotx = J（q）*dotq
%dotx是末端速度，q是电机位置，dotq是电机速度
% clear all
% clc

% syms q0 q1 q2 AC CD CE AG GH AJ H_0x B_0y PA_x PA_y LM BF GF k1 k2 
syms q0 q1 q2
% q0=mot_pos(1);
% q1=mot_pos(2);
% q2=mot_pos(3);

% q0=1;
% q1=1;
% q2=1;
AC = 185;
CD = 100;
AG = 100;
DE = 392.24;
% DE=375;
DG = 185;
GF = 60;
GH = 25;
AJ = 90.5;
LM = 20;
BF = 120;
CE=CD+DE;


%在x'Ay'坐标系下的坐标值
H_0x = -20;
B_0y = 69;


PA_x=48;
PA_y=32;
k1=-16*2.5/26/2/pi;
k2=19/50/28;

%未加说明，以下坐标表示均在x'Ay'坐标系下表示
% k1=16*2.5/26/2/pi;
% k2=19/50/28;
deltaX=k1*q0;
deltaY=k1*q1;
Hx=H_0x+deltaX;
Hy=-AJ;
Bx=LM;
By=B_0y-deltaY;
AH=sqrt(Hx^2+Hy^2);
a_GAH=acos((AG^2+AH^2-GH^2)/(2*AG*AH));
a_HAJ=atan(Hx/Hy);
a_GAJ=a_GAH-a_HAJ;
Gx=AG*sin(a_GAJ);
Gy=-AG*cos(a_GAJ);
BG=sqrt((Bx-Gx)^2+(By-Gy)^2);
a_BGF=acos((BG^2+GF^2-BF^2)/(2*BG*GF));
a_BGT=atan2(By-Gy,Bx-Gx);
a_FGT=a_BGT-a_BGF;
Fx=Gx+GF*cos(a_FGT);
Fy=Gy+GF*sin(a_FGT);
vec_AE=(AC/GF*[Fx-Gx,Fy-Gy]+CE/AG*[Gx,Gy]);

x_tilde=vec_AE(1);
y_tilde=vec_AE(2);
%x_tilde y_tilde 是在x'Ay'坐标系下的坐标

x0=x_tilde+PA_x;
y0=y_tilde-PA_y;
%x0 y0 是在xPy坐标系下的坐标

alpha=k2*q2;
x=x0/sqrt(1+(tan(alpha))^2);
y=y0;
z=x0/sqrt(1+(tan(alpha))^2)*tan(alpha);

%以上部分其实就是正解
%接下来要求导
% dot_Gx=gradient(Gx,[q0,q1,q2]);
% dot_Gy=gradient(Gy,[q0,q1,q2]);
dot_x=gradient(x,[q0,q1,q2]);
dot_y=gradient(y,[q0,q1,q2]);
dot_z=gradient(z,[q0,q1,q2]);


%d单腿雅可比矩阵为
xx=subs(dot_x,{q0,q1,q2},{mot_pos(1),mot_pos(2),mot_pos(3)});
yy=subs(dot_y,{q0,q1,q2},{mot_pos(1),mot_pos(2),mot_pos(3)});
zz=subs(dot_z,{q0,q1,q2},{mot_pos(1),mot_pos(2),mot_pos(3)});
xxx=double(xx);
yyy=double(yy);
zzz=double(zz);
Jac =[xxx,yyy,zzz]';




