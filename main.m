%main函数
%指定末端xyz移动到（az,bz,cz）,az,bz,cz是世界坐标系下的值
%单位mm
clear all
clc
az=-800;
bz=-280;
cz=-100;
%s是梯形曲线输出值，（0，1）
j=1;
for s=0:0.02:1
    [EE ,MOT]=EllipticCurve(az,bz,cz,s);
    i=1;
    Ee_Position(i,j)=EE(1);
    Mot_Pos(i,j)=MOT(1);
    i=i+1;
    Ee_Position(i,j)=EE(2);
    Mot_Pos(i,j)=MOT(2);
    i=i+1;
    Ee_Position(i,j)=EE(3);
    Mot_Pos(i,j)=MOT(3);
    j=j+1;
    %Ee_Position是在腿坐标系下，zxy
    %Mot_Pos对应rxy
end

t=0:0.02:1;
e1=Ee_Position(1,:);
e2=Ee_Position(2,:);
e3=Ee_Position(3,:);
m1=Mot_Pos(1,:);
m2=Mot_Pos(2,:);
m3=Mot_Pos(3,:);
writematrix('')
%%
%画出驱动构件的输出曲线
plot(t,m1,'red');
hold on
plot(t,m2,'black');
hold on
plot(t,m3,'blue');
hold on
legend('m1--Rotation','m2--X','m3--Y');
xlabel('s(t)');
ylabel('m');
title('Mot\_Pos')
%%
%画末端轨迹的三维图，在（固定）腿坐标系下
% plot3(e2,e3,e1);
% 
% xlabel('x');
% ylabel('y');
% zlabel('z');
% grid on
