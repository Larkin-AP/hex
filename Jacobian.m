%计算雅可比矩阵
%X = J*theta
%X是末端位置，theta是电机位置
%目前theta输入是[l1.l2.alpha],输出是末端位置X=[x,y,z]
clear all
clc

syms q0 q1 q2 AC CD CE AG GH AJ H_0x B_0y PA_x PA_y LM BF GF k1 k2

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
a_HAJ=atan(Hx,Hy);
a_GAJ=a_GAH-a_HAJ;
Gx=AG*sin(a_GAJ);
Gy=-AG*cos(a_GAJ);
BG=sqrt((Bx-Gx)^2+(By-Gy)^2);
a_BGF=acos((BG^2+GF^2-BF^2)/(2*BG*GF));
a_BGT=atan2(By-Gy,Bx-Gx);
a_FGT=a_BGT-a_BGF;
Fx=Gx+GF*cos(a_FGT);
Fy=Gy+GF*sin(a_FGT);
vec_AE=AC/GF*[Fx-Gx,Fy-Gy]+CE/AG*[Gx,Gy];

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
dot_Gx=gradient(Gx,[q0,q1,q2]);
dot_Gy=gradient(Gy,[q0,q1,q2]);
dot_x=gradient(x,[q0,q1,q2]);
dot_y=gradient(y,[q0,q1,q2]);
dot_z=gradient(z,[q0,q1,q2]);
% dot_x=simplify(dot_x);
% dot_y=simplify(dot_y);
% dot_z=simplify(dot_z);
%d单腿雅可比矩阵为

dot_x=subs(dot_x,{q0,q1,q2,AC,CD,CE,AG,GH,AJ,H_0x,B_0y,PA_x,PA_y,LM,BF,GF,k1,k2},{0,0,0,185,100,475,100,25,90.5,7.105,44.7557,48,32,20,120,60,16*2.5/26/2/pi,19/50/28});
% dot_x=roundn(dot_x,-4);




