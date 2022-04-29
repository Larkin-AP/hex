% %% check 
clear all
clc
% 
% log = readmatrix('hex_forward(check).txt');
% m4 = log(:,1);
% m5 = log(:,2);
% m6 = log(:,3);
% ee_fx = log(:,4);
% ee_fy = log(:,5);
% ee_fz = log(:,6);
% ee_bx = log(:,7);
% ee_by = log(:,8);
% ee_bz = log(:,9);
% t=0.001:0.001:size(log(:,1))/1000;
% t1=t(1:end-1);
% 
% diff_m4 = m4(2:end)-m4(1:end-1);
% diff_m5 = m5(2:end)-m5(1:end-1);
% diff_m6 = m6(2:end)-m6(1:end-1);
% 
% ee_relx = ee_fx-ee_bx;
% ee_rely = ee_fy-ee_by;
% ee_relz = ee_fz-ee_bz;
% 
% dif_relx = ee_relx(2:end)-ee_relx(1:end-1);
% dif_rely = ee_rely(2:end)-ee_rely(1:end-1);
% dif_relz = ee_relz(2:end)-ee_relz(1:end-1);
% 
% J = Jacob([m4(100),m5(100),m6(100)]);
% xx = J*[diff_m4(100),diff_m5(100),diff_m6(100)]'
% dif = [dif_relx(100),dif_rely(100),dif_relz(100)]

%% 读取数据
% motor = readmatrix('leg1MotorPos1.txt');%电机位置单位为rad，采样频率为1000Hz  ,沿x方向移动
% motor = readmatrix('leg1MotorPos.txt'); %沿z方向运动
motor = readmatrix('leg1MotorPos2.txt'); %原地转向
q0=motor(2:end,1);
q1=motor(2:end,2);
q2=motor(2:end,3);

%速度要转换单位
dq0=motor(2:end,1)-motor(1:end-1,1);
dq1=motor(2:end,2)-motor(1:end-1,2);
dq2=motor(2:end,3)-motor(1:end-1,3);
dq0=dq0*1000;  %速度单位为rad/s
dq1=dq1*1000;
dq2=dq2*1000;


% endTraj=readmatrix('leg1EndTraj1.txt');
% %这些是世界坐标系下的坐标，最终要得到腿坐标系下末端的速度，故需转换坐标系  单位m  沿x方向运动
% endTraj=readmatrix('leg1EndTraj.txt');  %沿z方向运动
endTraj=readmatrix('leg1EndTraj2.txt');  %原地转向
ee_in_ground = endTraj(1:end,17:19);
ground_P_body = endTraj(1:end,1:16);
ee_in_leg=zeros(size(ee_in_ground,1),3); %只有x方向坐标做了平移
%先得到腿坐标系下的末端坐标
for i=1:size(ee_in_ground,1)
    ee_in_leg(i,:)=coordiTrans(ee_in_ground(i,:),ground_P_body(i,:));
end

% aa=coordiTrans(ee_in_ground(1000,:),ground_P_body(1000,:));

% 计算末端速度   单位m/s  规划理论值

Vx=(ee_in_leg(2:end,1)-ee_in_leg(1:end-1,1))*1000;
Vy=(ee_in_leg(2:end,2)-ee_in_leg(1:end-1,2))*1000;
Vz=(ee_in_leg(2:end,3)-ee_in_leg(1:end-1,3))*1000;

    



 



%% 输入和函数计算






mot_pos=[q0 q1 q2];
dq=[dq0 dq1 dq2];
v=zeros(size(dq,1),3);
lambda=zeros(size(dq,1),3); %特征值
mu1=zeros(size(dq,1),1);  %各向同性 为条件数开根号 ，mu1越接近1，各向同性越好，趋近无穷大，则机构趋于奇异

mu2=zeros(size(dq,1),1);  %条件数

for i=1:size(mot_pos,1)
    J=CalJac(mot_pos(i,:));
    A=J*J';
    lambda(i,:)=eig(A)';
    mu2(i,1)=max(lambda(i,:))/min(lambda(i,:));
%     mu1(i,1)=sqrt(mu2(i,1));
    mu1(i,1) = 1/mu2(i,1);
    v(i,:)=(J*dq(i,:)')'; % v是末端xyz速度，腿坐标系下
end



% J=CalJac(mot_pos(350,:));






%% plot
t=0.001:0.001:size(v,1)/1000;

%通过雅可比计算出来的末端速度
figure(1)
plot(t,v(:,1),'r',t,Vx,'--g');
legend('J*dq','Vx');
title('理论速度和通过雅可比矩阵计算速度对比-Vx')

figure(2)
plot(t,v(:,2),'r',t,Vy,'--g');
legend('J*dq','Vy');
title('理论速度和通过雅可比矩阵计算速度对比-Vy')

figure(3)
plot(t,v(:,3),'r',t,Vz,'--g');
legend('J*dq','Vz');
title('理论速度和通过雅可比矩阵计算速度对比-Vz')

%分开画
figure(4)
subplot(3,3,1);
plot(t,dq0);
title('dq0');

subplot(3,3,2);
plot(t,dq1);
title('dq1');

subplot(3,3,3);
plot(t,dq2);
title('dq2');

subplot(3,3,4);
plot(t,v(:,1));
title('J*dq - x');

subplot(3,3,5);
plot(t,v(:,2));
title('J*dq - y');

subplot(3,3,6);
plot(t,v(:,3));
title('J*dq - z');

subplot(3,3,7);
plot(t,Vx);
title('Vx');

subplot(3,3,8);
plot(t,Vy);
title('Vy');

subplot(3,3,9);
plot(t,Vz);
title('Vz');
% 
% %画腿坐标系下的末端轨迹，对比末端速度是否合理
figure(5);
subplot(1,3,1);
plot(t,ee_in_leg(1:end-1,1));
legend('ee\_in\_leg');
title('X');

subplot(1,3,2);
plot(t,ee_in_leg(1:end-1,2));
legend('ee\_in\_leg');
title('Y');

subplot(1,3,3);
plot(t,ee_in_leg(1:end-1,3));
legend('ee\_in\_leg');
title('z');

%画世界坐标系下的末端轨迹
figure(6)
subplot(1,3,1);
plot(t,ee_in_ground(1:end-1,1),t,ee_in_leg(1:end-1,1),'--g');
legend('ee\_in\_ground','ee\_in\_leg');
title('X');

subplot(1,3,2);
plot(t,ee_in_ground(1:end-1,2),t,ee_in_leg(1:end-1,2),'--g');
legend('ee\_in\_ground','ee\_in\_leg');
title('Y');

subplot(1,3,3);
plot(t,ee_in_ground(1:end-1,3),t,ee_in_leg(1:end-1,3),'--g');
legend('ee\_in\_ground','ee\_in\_leg');
title('Z');

%画条件数
figure(7)
plot(t,mu2);
title('条件数');

figure(8)
plot(t,mu1);
title('各向同性');

%% 计算最大速度

%Rb是腿在身体坐标系下的旋转矩阵
 Rb = [1,0,0;
     0,1,0;
     0,0,1];

 Vn=2/3*100*pi; %电机额定转速  单位rad/s

 eb=[1,0,0]'; %机身坐标系下指定方向（沿x方向）
 ve=zeros(size(mot_pos,1),1);
 
for i=1:size(mot_pos,1)
    J=CalJac(mot_pos(i,:));
    Jb=Rb*J; %机身坐标系下的雅可比矩阵
    temp=(inv(Jb))*eb;
    ve(i,1)=Vn/norm(temp,Inf);

end
plot(t,ve);
title('x方向最大速度  m/s') %最大速度在0.471-0.466m/s之间 100pi

%2000rpm  在0.311-0.314m/s之间









%% 必要参数计算（正解）
%雅可比矩阵单位用m作为单位计算的
function [J]=CalJac(mot_pos)

q0=mot_pos(1);
q1=mot_pos(2);
q2=mot_pos(3);


%正解
AC = 0.185;
CD = 0.100;
AG = 0.100;
DE = 0.39224;
% DE=0.375;
DG = 0.185;
GF = 0.060;
GH = 0.025;
AJ = 0.0905;
LM = 0.020;
BF = 0.120;
CE=CD+DE;
%固定角度
%x向初始位置,注意这个初始位置 不是任一点 是H点

%y向初始位置,即P点比A点在y方向高出的长度

% H_0x = 7.105;%坐标值
% B_0y = 44.7557;

%在x'Ay'坐标系下的坐标值
% H_0x = -0.020;
% B_0y = 0.069;

H_0x=0.007105;
B_0y=0.0447558;


PA_x=0.048;
PA_y=0.032;
% mot_pos=[-318.5526,310.3820,-64.2526]; %电机输入量，目前是二维，分别为xy方向电机输入量


k1=16*0.0025/26/2/pi;
k2=19/50/28;

deltaX=-k1*q0;
deltaY=k1*q1;
alpha=k2*q2;


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

x=x0/sqrt(1+(tan(alpha))^2);
y=y0;
z=x0*(tan(alpha))/sqrt(1+(tan(alpha))^2);



% 计算自己的雅可比矩阵
% theta1=a_FGT;
% theta2=a_GAJ;
% theta3=a_BGT;
% theta4=a_BGF;
% theta5=a_GAH;
% theta6=a_HAJ;





theta3=angle_BGT;
theta4=angle_BGF;
theta6=angle_HAJ;
theta5=angle_GAH;
theta2=angle_GAJ;
theta1=angle_FGT;


m1=(k1*(cos(theta3))^2)/(Bx-Gx);
n1=(((By-Gy)*cos(theta2)-(Bx-Gx)*sin(theta2))*AG*(cos(theta3))^2)/(Bx-Gx)^2;


m2=((GF^2+BG^2-BF^2)/(2*GF*BG^2)-1/GF)/sqrt(1-(GF^2+BG^2-BF^2)^2/(4*GF^2*BG^2))*(k1*(By-Gy)/(sqrt((Bx-Gx)^2+(By-Gy)^2)));
n2=((GF^2+BG^2-BF^2)/(2*GF*BG^2)-1/GF)/sqrt(1-(GF^2+BG^2-BF^2)^2/(4*GF^2*BG^2))*(-((By-Gy)*sin(theta2)+(Bx-Gx)*cos(theta2))*AG/(sqrt((Bx-Gx)^2+(By-Gy)^2)));


m3=((AG^2+AH^2-GH^2)/(2*AG*AH^2)-1/AG)/sqrt(1-(AG^2+AH^2-GH^2)^2/(4*AG^2*AH^2))*((-Hx*k1)/sqrt(Hx^2+Hy^2));
m4=-k1/(Hy*((Hx^2/Hy^2)+1));



% dtheta5=m3*dq0;
% dtheta6=m4*dq0;
% dtheta2=dtheta5-dtheta6;
% dtheta4=m2*dq1+n2*dtheta2;
% dtheta3=m1*dq1+n1*dtheta2;
% dtheta1=dtheta3-dtheta4;

a1=1/sqrt(1+(tan(alpha))^2)*((CE*cos(theta2)-AC*sin(theta1)*(n1-n2))*(m3-m4));
b1=1/sqrt(1+(tan(alpha))^2)*(-AC*sin(theta1)*(m1-m2));
c1=-(x0*tan(alpha)*k2)/sqrt(1+(tan(alpha))^2);

a2=((AC*cos(theta1)*(n1-n2)+CE*sin(theta2))*(m3-m4));
b2=AC*cos(theta1)*(m1-m2);

a3=tan(alpha)/sqrt(1+(tan(alpha))^2)*((CE*cos(theta2)-AC*sin(theta1)*(n1-n2))*(m3-m4));
b3=tan(alpha)/sqrt(1+(tan(alpha))^2)*(-AC*sin(theta1)*(m1-m2));
c3=(x0*sqrt(1+(tan(alpha))^2)-(x0*(tan(alpha))^2)/sqrt(1+(tan(alpha))^2))*k2;

% vx=vpa(a1*dq0+b1*dq1+c1*dq2,5);
% vy=vpa(a2*dq0+b2*dq1+0*dq2,5);
% vz=vpa(a3*dq0+b3*dq1+c3*dq2,5);

J=[a1 b1 c1;a2 b2 0;a3 b3 c3];

end


%% 坐标变换函数
function [PL]=coordiTrans(ee_in_ground,ground_P_body) %传入末端在世界坐标系下的坐标，得到末端在腿坐标系下的坐标
% 转换公式： Leg_xyz_ee =Leg_P_Body * Body_P_Ground * Ground_xyz_ee(input4*1)
PL1=[1, 0,  0,  -0.335;
	 0, 1,  0,  0;
	 0, 0,  1,	0;
	 0, 0,  0,  1]; %身体在腿坐标系下的变换矩阵  Leg_P_Body
 
PL2 = reshape(ground_P_body,[4,4])'; %Ground_P_Body
 %Body_P_Ground
PL3 = PL1/PL2; %Leg_P_Body * Body_P_Ground = Leg_P_Ground
temp=[reshape(ee_in_ground,[3,1]);1];
PL4 = PL3 * temp; %Leg_P_Ground* Ground_P_ee = Leg_P_ee
PL = PL4(1:3,1);



end



