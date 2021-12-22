function [ee_position]=Forward_kinematics(mot_pos)
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
% DE = 392.27;
DE=375;
DG = 185;
GF = 60;
GH = 25;
AJ = 90.5;
LM = 20;
BF = 120;
CE=CD+DE;
%固定角度
%x向初始位置,注意这个初始位置 不是任一点 是H点
%H_0x = 40;
%y向初始位置,即P点比A点在y方向高出的长度
%B_0y = 58;
H_0x = 7.105;%坐标值
B_0y = 44.7557;
PA_x=48;
PA_y=32;
% mot_pos=[0,0,0]; %电机输入量，目前是二维，分别为xy方向电机输入量




deltaX=16*2.5/26/2/pi*mot_pos(1);
deltaY=16*2.5/26/2/pi*mot_pos(2);
Hx=H_0x+deltaX;
Hy=-AJ;
Bx=LM;
By=B_0y-deltaY;
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
alpha=19/50/28*mot_pos(3);
x=x0/sqrt(1+(tan(alpha))^2);
y=y0;
z=x0/sqrt(1+(tan(alpha))^2)*tan(alpha);
ee_position=[x,y,z];








