function [mot_pos3 ]=Inverse_kinematic(ee_position)
% clear all
% clc
%注意：这里的反解结果是没问题的，但是q0,q1的符号与实际使用的相反%
%重新修改 3d版本
% ee_position为末端坐标，此处要求传入末端在腿坐标系的位置，按顺序为xyz
% mot_pos即为驱动杆件的坐标变化值,按顺序为x,y,Rotation
%末端在初始位置的xyz为[256.8 -360.9 0]
%[304.77 -392.91 0]
%竖直转动轴
% ee_position = [347.49,-372.52,0];
theta0 = atan2(ee_position(3),ee_position(1));
q2 = 50 * 28 / 19 * theta0; 
x0=sqrt(ee_position(3)^2+ee_position(1)^2);
y0=ee_position(2);
PA_x=48;
PA_y=32;

x=x0-PA_x;
y=y0+PA_y;
%末端位置x,y
%固定杆长
AC = 185;
CD = 100;
AG = 100;
% DE = 392.27;
DE=375;
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
H_0x = 7.105;%坐标值
B_0y = 44.7557;



CE = CD + DE;
AE = sqrt(x^2+y^2);
angle_ECA = (acos((CE^2+AC^2-AE^2)/2/CE/AC));
angle_CAE = (acos((AC^2+AE^2-CE^2)/2/AC/AE));
angle_CAG = pi - angle_ECA;
angle_EAG = angle_CAG-angle_CAE;
angle_EAJ = (atan(-x/y));
angle_GAJ = angle_EAJ - angle_EAG;
Gx = AG*sin(angle_GAJ);
Gy = -AG*cos(angle_GAJ);
vector_AG = [Gx  Gy];
Hy =-AJ;

%两种方法都可以
% GK=abs(Gy-Hy);
% angle_GHK=asin(GK/GH);
% HK=GH*cos(angle_GHK);
% Hx = Gx-HK;

HN = sqrt(GH^2- (Gy-Hy)^2);
Hx = Gx - HN;

deltaX = -(Hx - H_0x);
q0 =- 26.0/16.0*deltaX /2.5*2.0*pi;%X电机

vector_AE = [x y];
vector_CE = CE/AG*vector_AG;
vector_AC = vector_AE-vector_CE;
vector_GF = GF/AC*vector_AC;
vector_AF = vector_AG+vector_GF;

FL = vector_AF(1)-LM;
BL = sqrt(BF^2-FL^2);
By = vector_AF(2)+BL;
deltaY = (By-B_0y);
q1 = -26.0 / 16.0 * deltaY / 2.5 * 2.0 * pi;
mot_pos3=[q0,q1,q2];
%此处打印的是腿坐标系下的位置
%fprintf('the ee_position is (zxy)[%8.8f  %8.8f,  %8.8f],the mot_pos3 is (rxy)[%8.8f  %8.8f,  %8.8f]\n',ee_position(1),ee_position(2),ee_position(3),mot_pos3(1),mot_pos3(2),mot_pos3(3));
% Bx=LM;
% BG=sqrt((Bx-Gx)^2+(By-Gy)^2);
% angle_BGF=acos((BG^2+GF^2-BF^2)/(2*BG*GF));
% angle_BGF


