%画可操作度
%{
输入腿坐标系下的坐标
求反解，获得电机角度
再把电机角度传给计算雅可比矩阵
计算雅可比矩阵特征值和特征向量
根据特征值和特征向量画图
%}


clear all
clc

%传入坐标
xyz_coord = [0.5665,-0.4667,0.0081];
%计算反解
q = Inverse_kinematic(xyz_coord);
%计算雅可比
J = CalJac(q);

%计算特征值和特征向量
A = J*J';
[V,D] = eig(A); %V的各列是对应的特征向量，D的对角值为特征值
D=diag(D);

% 椭球可视化
%椭球体积
V_ellipsoid = sqrt(det(A)); %此处是正比关系，并不是等号
D = sqrt(D)';

gtEllip=ellipsoidalFit.groundtruth([],[0,0,0],D,[0,0,0]);
gtEllip.R=V;

subplot(2,2,1);
plot(gtEllip);
view(25,30);

subplot(2,2,2);
plot(gtEllip);
view(0,0);

subplot(2,2,3);
plot(gtEllip);
view(0,90);

subplot(2,2,4);
plot(gtEllip);
view(90,0);

suptitle('Degree of Operation');


