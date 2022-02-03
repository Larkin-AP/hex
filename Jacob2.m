%计算雅可比矩阵符号解
% clear all
% clc
% ee_position为末端坐标，此处要求传入末端在腿坐标系的位置，按顺序为xyz
% mot_pos即为驱动杆件的坐标变化值,按顺序为x,y,Rotation
%末端在初始位置的xyz为[256.8 -360.9 0]
%[304.77 -392.91 0]
%竖直转动轴
% ee_position = [304.77 -392.91 0];
% mot_pos3(3)=atan2(ee_position(3),ee_position(1));%此处暂时加负号，方便理解,记录的是角度
% x0=sqrt(ee_position(3)^2+ee_position(1)^2);
% y0=ee_position(2);
% 
% x=x0-48;
% y=y0+32;
%末端位置x,y
%固定杆长
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
%固定角度
%x向初始位置,注意这个初始位置 不是任一点 是H点

%y向初始位置,即P点比A点在y方向高出的长度

% H_0x = 7.105;%坐标值
% B_0y = 44.7557;

%在x'Ay'坐标系下的坐标值
H_0x = -20;
B_0y = 69;

PA_x=48;
PA_y=32;
% mot_pos=[-318.5526,310.3820,-64.2526]; %电机输入量，目前是二维，分别为xy方向电机输入量




deltaX=-16*2.5/26/2/pi*mot_pos(1);
deltaY=-16*2.5/26/2/pi*mot_pos(2);
Hx=H_0x+deltaX;
Hy=-AJ;
Bx=LM;
By=B_0y+deltaY;
AH=sqrt(Hx^2+Hy^2);
angle_GAH=acos((AG^2+AH^2-GH^2)/(2*AH*AG));
angle_HAJ=atan(Hx/Hy);
angle_GAJ=angle_GAH-angle_HAJ;
Gx=AG*sin(angle_GAJ);
Gy=-AG*cos(angle_GAJ);
BG=sqrt((Bx-Gx)^2+(By-Gy)^2);
angle_BGT=atan2(By-Gy,Bx-Gx);
angle_BGF=acos((BG^2+GF^2-BF^2)/(2*BG*GF));
angle_FGT=angle_BGT-angle_BGF;
Fx=Gx+GF*cos(angle_FGT);
Fy=Gy+GF*sin(angle_FGT);
vector_GF=[GF*cos(angle_FGT),GF*sin(angle_FGT)];
vector_AG=[Gx,Gy];
vector_AE=AC/GF*vector_GF+CE/AG*vector_AG;
x_tilde=vector_AE(1);
y_tilde=vector_AE(2);
x0=x_tilde+PA_x;
y0=y_tilde-PA_y;
%电机的弧度转化到到平面的弧度
alpha=19/50/28*mot_pos(3);
x=x0/sqrt(1+(tan(alpha))^2);
y=y0;
z=x0/sqrt(1+(tan(alpha))^2)*tan(alpha);
ee_position=[x,y,z];


%%
clear all
clc
syms q0 q1 q2 k1 k2 AC CD AG DE GF GH AJ LM BF CE H_0x B_0y PA_x PA_y
alpha= k2*q2;
deltaX=k1*q0;
deltaY=k1*q1;


Hx=H_0x+deltaX;
Hy=-AJ;
Bx=LM;
By=B_0y+deltaY;
AH=sqrt((H_0x+deltaX)^2+Hy^2);




the5=acos((AG^2+AH^2-GH^2)/(2*AG*AH));
the6=atan((H_0x+deltaX)/Hy);
the2=the5-the6;
Gx=AG*sin(the2);
Gy=-AG*cos(the2);

the3=atan2(By-Gy,Bx-Gx);
temp=gradient(((By-Gy)/(Bx-Gx)),[q0,q1,q2]);
dthe3=temp*(cos(the3))^2;

BG = sqrt((Bx-Gx)^2+(By-Gy)^2);
the4=acos((BG^2+GF^2-BF^2)/(2*BG*GF));

the1=the3-the4;
dthe4=gradient(the4,[q0,q1,q2]);
dthe1=dthe3-dthe4;

dthe5=gradient(the5,[q0,q1,q2]);
dthe6=gradient(the6,[q0,q1,q2]);
dthe2=dthe5-dthe6;

dalpha=gradient(alpha,[q0,q1,q2]);

x_tilde=AC*cos(the1)+CE*sin(the2);
y_tilde=AC*sin(the1)-CE*cos(the2);

x0=x_tilde+PA_x;
dx0=-AC*sin(the1)*dthe1+CE*cos(the2)*dthe2;



%% 
clear all
clc
syms q0 q1 q2 k1 k2 AC CD AG DE GF GH AJ LM BF CE H_0x B_0y PA_x PA_y the5 the6 the2

deltaY=k1*q1;
By=B_0y+deltaY;
Bx=LM;
Hy=-AJ;
alpha= k2*q2;
deltaX=k1*q0;

AH=sqrt((H_0x+deltaX)^2+Hy^2);
the5=acos((AG^2+AH^2-GH^2)/(2*AG*AH));
the6=atan((H_0x+deltaX)/Hy);
the2=the5-the6;
dthe5=gradient(the5,[q0,q1,q2]);
dthe6=gradient(the6,[q0,q1,q2]);
dthe2=dthe5-dthe6;
Gx=AG*sin(the2);
Gy=-AG*cos(the2);
temp=gradient(((By-Gy)/(Bx-Gx)),[q0,q1,q2]);
a=simplify(temp);



%% the5
syms AG AH GH
the5=acos((AG^2+AH^2-GH^2)/(2*AG*AH));
dthe5=gradient(the5,AH);

%% dAH
clear all
clc
syms q0 q1 q2 H_0x k1 AJ

Hy=-AJ;
deltaX=k1*q0;
AH=sqrt((H_0x+deltaX)^2+Hy^2);
dAH=gradient(AH,[q0,q1,q2]);

%% the6
clear all
clc

syms q0 q1 q2 H_0x k1 AJ
Hy=-AJ;
deltaX=k1*q0;
the6=atan((H_0x+deltaX)/Hy);
dthe6=gradient(the6,[q0,q1,q2]);


%% the5
clear all 
clc
syms BG GF BF
the4=acos((BG^2+GF^2-BF^2)/(2*BG*GF));
dthe4=gradient(the4,BG);

%% BG


BG=sqrt((Bx-Gx)^2+(By-Gy)^2);







