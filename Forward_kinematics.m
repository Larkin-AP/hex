function [ee_position]=Forward_kinematics(mot_pos)
clear all
clc
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
DE = 155;
EN = 220;
DG = 185;
GF = 60;
GH = 25;
AJ = 90.5;
LM = 20;
BF = 120;
%固定角度
%x向初始位置,注意这个初始位置 不是任一点 是H点
%H_0x = 40;
%y向初始位置,即P点比A点在y方向高出的长度
%B_0y = 58;
H_0x = -2;%坐标值
B_0y = 44;
mot_pos=[0,0,0]; %电机输入量，目前是二维，分别为xy方向电机输入量



NC = CD + DE +EN;
Hx=H_0x+mot_pos(1);
Hy=-AJ;
Bx=LM;
By=B_0y+mot_pos(2);
AH=sqrt(Hx^2+Hy^2);
angle_AHG=acos((AH^2+GH^2-AG^2)/(2*AH*GH));
% if (Hx<0)
%     angle_AHK=atan(abs(Hy/Hx));
% elseif (Hx>0)
%     angle_AHK=pi-atan(abs(Hy/Hx));
% else
%     angle_AHK=pi/2;
% end
angle_AHK=atan2(-Hy,Hx);
angle_GHR=pi-angle_AHK-angle_AHG;
HK=GH*cos(angle_GHR);
GK=GH*sin(angle_GHR);
Gx=Hx+HK;
Gy=Hy+GK;
vector_AG=[Gx,Gy];
% if (Bx>Gx)
%     angle_BGR=atan(abs((By-Gy)/(Bx-Gx)));
% elseif (Bx<Gx)
%     angle_BGR=pi-atan(abs((By-Gy)/(Bx-Gx)));
% else
%     angle_BGR=pi/2;
% end
angle_BGT=atan2(By-Gy,Bx-Gx);
BG=sqrt((Bx-Gx)^2+(By-Gy)^2);
angle_BGF=acos((BG^2+GF^2-BF^2)/(2*BG*GF));
angle_FGT=angle_BGT-angle_BGF;
FR=GF*sin(angle_FGT);
GR=GF*cos(angle_FGT);
Fx=Gx+GR;
Fy=Gy+FR;
vector_GF=[Fx-Gx,Fy-Gy];
vector_AC=AC/GF*vector_GF;
vector_CN=NC/AG*vector_AG;
vector_AN=vector_AC+vector_CN;
x=vector_AN(1);
y=vector_AN(2);
y0=y-32;
syms x0 z0;
eq1=sqrt(x0^2+z0^2)-x-48;
eq2=tan(mot_pos(3))-z0/x0;
s=solve(eq1,eq2);
x0=s.x0(1);
z0=s.z0(1);
digits(5);
ee_position=[vpa(x0),vpa(y0),vpa(z0)];





