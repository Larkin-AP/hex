function [mot_pos3 ]=Inverse_kinematic(ee_position)
clear all
clc
% ee_position为末端坐标，此处要求传入末端在腿坐标系的位置，按顺序为xyz
% mot_pos即为驱动杆件的坐标变化值,按顺序为x,y,Rotation
%末端在初始位置的xyz为[256.8 -360.9 0]
%[304.77 -392.91 0]
%竖直转动轴
ee_position = [304.77 -392.91 0];
mot_pos3(3)=atan2(ee_position(3),ee_position(1));%此处没负号，方便理解,记录的是角度
x0=sqrt(ee_position(3)^2+ee_position(1)^2);
y0=ee_position(2);

x=x0-48;
y=y0+32;
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



NC = CD + DE +EN;
AN = sqrt(x^2+y^2);
angle_NCA = (acos((NC^2+AC^2-AN^2)/2/NC/AC));
angle_CAN = (acos((AC^2+AN^2-NC^2)/2/AC/AN));
angle_CAG = pi - angle_NCA;
angle_NAG = angle_CAG-angle_CAN;
angle_NAJ = (atan(-x/y));
angle_GAJ = angle_NAJ - angle_NAG;
Gx = AG*sin(angle_GAJ);
Gy = -AG*cos(angle_GAJ);
vector_AG = [Gx  Gy];
Hy =-AJ;

GK=abs(Gy-Hy);
angle_GHK=asin(GK/GH);
HK=GH*cos(angle_GHK);
Hx = Gx-HK;

% if Gy<Hy
%     HK = Hy-Gy;
%     angle_HGK = asin(HK/GH);
%     GK = GH*cos(angle_HGK);
%     Hx = Gx - GK;
% elseif Gy>Hy
%     GK = Gy-Hy;
%     angle_GHK=asin(GK/GH);
%     HK = GH*cos(angle_GHK);
%     Hx = Gx-HK;
% else
%     Hx= Gx-GH;
% end
mot_pos3(1) = Hx-H_0x;%X推杆变化距离

vector_AN = [x y];
vector_CN = NC/AG*vector_AG;
vector_AC = vector_AN-vector_CN;
vector_GF = GF/AC*vector_AC;
vector_AF = vector_AG+vector_GF;
m=vector_AF(1);
n=vector_AF(2);
FL = m-LM;
BL = sqrt(BF^2-FL^2);
By = n+BL;
mot_pos3(2) = By-B_0y;
mot_pos3
%此处打印的是腿坐标系下的位置
%fprintf('the ee_position is (zxy)[%8.8f  %8.8f,  %8.8f],the mot_pos3 is (rxy)[%8.8f  %8.8f,  %8.8f]\n',ee_position(1),ee_position(2),ee_position(3),mot_pos3(1),mot_pos3(2),mot_pos3(3));




