function [ee ,mot]=EllipticCurve(az,bz,cz,s)
%elliptic curve
%a0,b0,c0是末端初始位置
%az,bz,cz是要求末端运动到的位置，世界坐标系
a0=-677.96;
b0=-348.51;
c0=0;
%求椭圆轨迹的参数
a1=az-a0;
b1=bz-b0;
c1=cz-c0;
%s相当于t形曲线的输出值
%此处坐标皆为世界坐标系
xt = a0+a1*(1+cos(pi-pi*s))/2;
yt = b0+b1*sin(pi-pi*s);
zt = c0+c1*(1+cos(pi-pi*s))/2;
%调用反解函数需要转换到腿坐标系下
G_X_ee = [xt,yt,zt,1]';
B_P_G = eye(4);
L_P_B = [1 0 0 318;
         0 1 0 43;
         0 0 1 0;
         0 0 0 1];
L_X_ee = L_P_B*B_P_G*G_X_ee;
%注意此处传入的是xyz
ee =[L_X_ee(1),L_X_ee(2),L_X_ee(3)]; %ee_position是xyz,且是腿坐标系下的
mot=Inverse_kinematic([L_X_ee(1),L_X_ee(2),L_X_ee(3)]);
