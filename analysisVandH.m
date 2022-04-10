%计算运动速度和涉水能力的关系
%{
注释区
涉水能力实际就是高度h，详细点说就是电机以下的位置高度h
运动速度v，此处以理论上可达到的最大速度进行计算vmax计算，这个目前有公式
我们可以遍历整个三维空间，在工作空间内确定每个点的最大速度
找到一个合适的工作空间
何为合适的工作空间？
此处目前先猜想，该空间足够大，速度适中（尽量快）
确定出该处的工作空间后再确定高度h
再做衡量

方法二：设计一个函数f=f(v,h)
最简单的形式f=av+bh，a和b是两个系数，这个系数可以根据经验去设立
再写具体点,此处v用vmax代替,q是单腿的关节位置，q=q(q0,q1,q2)
f=av+bh=av(q)+bh(q)=f(q)

%}



%% 按切片方式计算函数
%想办法把res转换成网格
%{
以q0，q1，q2的元素个数分别为21，6，11的情况下进行说明
我们固定y，网格平面为x*z的平面
即相当于有6张纸，每张纸为21*11的网格，每个网格对应其中的值
res应该是21*
%}


clear all
clc

[x_list,z_list] = meshgrid(0.165:0.01:0.580,-0.444:0.01:0.444);
[m,n]=size(x_list);
%Rb是腿在身体坐标系下的旋转矩阵
 Rb = [1,0,0;
     0,1,0;
     0,0,1];

 Vn=2/3*100*pi; %电机额定转速  单位rad/s

 eb_x=[1,0,0]'; %机身坐标系下指定方向（沿x方向）
 eb_y=[0,1,0]'; %机身坐标系下指定方向（沿y方向）
 eb_z=[0,0,1]'; %机身坐标系下指定方向（沿z方向）

h1=figure;
h2=figure;
h3=figure;

for y=-0.586:0.05:-0.215
    y_list=ones(m,n)*y;
    vel_list_x =nan(m,n);
    vel_list_y =nan(m,n);
    vel_list_z =nan(m,n);
    for i=1:m
        for j=1:n
            %[x_list(i,j),y,z_list(i,j)]取出来该点
            %先判断该点是否在工作空间内
            %用运动学反解，如果是复数，直接跳出本次循环
            %如果是实数，继续内容
            %运动学反解求解出关节空间坐标
            %根据关节空间求解雅可比矩阵和最大速度
            %把速度赋值给vel_list即可完成本次循环
            q=IKM([x_list(i,j),y,z_list(i,j)]); %q是向量
            if isreal(q) %q为实数，说明该点为工作空间，进入循环
                J=CalJac(q);
                Jb=Rb*J;
                temp_x=(inv(Jb))*eb_x;
                temp_y=(inv(Jb))*eb_y;
                temp_z=(inv(Jb))*eb_z;
                ve_x=Vn/norm(temp_x,Inf);
                ve_y=Vn/norm(temp_y,Inf);
                ve_z=Vn/norm(temp_z,Inf);
                vel_list_x(i,j) =ve_x; 
                vel_list_y(i,j) =ve_y;    
                vel_list_z(i,j) =ve_z;    
            end
                      
        end
    end
    
    figure(h1);
    hold on;
    surf(x_list,z_list,y_list,vel_list_x,'EdgeColor','none');  
    
    figure(h2);
    hold on;
    surf(x_list,z_list,y_list,vel_list_y,'EdgeColor','none');    
    
    figure(h3);
    hold on;
    surf(x_list,z_list,y_list,vel_list_z,'EdgeColor','none');
    
end

figure(h1);
set(gcf,'Units','centimeters','Position',[5 5 16 9]); %指定plot输出图片的尺寸，xmin，ymin，width，height
set(gca,'DataAspectRatio',[1,1,1],'PlotBoxAspectRatio',[1,1,1]...,'xLim',[0,1.2],'yLim',[-0.8,0.8],'zLim',[-0.7,0]...
    ...,'xtick',0.3:0.1:1.2,'ytick',-0.6:0.1:0.6,'ztick',-0.7:0.1:0 ...
    ...,'xgrid','on','ygrid','on','zgrid','on'...
    ,'yDir','reverse');
view(-50,30);
xlabel('X(m)');
ylabel('Z(m)');
zlabel('Y(m)');
title('Velocity distribution in the x direction')
caxis([0,1]);
colorbar;

figure(h2);
set(gcf,'Units','centimeters','Position',[5 5 16 9]);
set(gca,'DataAspectRatio',[1,1,1],'PlotBoxAspectRatio',[1,1,1]...,'xLim',[0,1.2],'yLim',[-0.8,0.8],'zLim',[-0.7,0]...
    ...,'xtick',0.3:0.1:1.2,'ytick',-0.6:0.1:0.6,'ztick',-0.7:0.1:0 ...
    ...,'xgrid','on','ygrid','on','zgrid','on'...
    ,'yDir','reverse');
view(-50,30);
xlabel('X(m)');
ylabel('Z(m)');
zlabel('Y(m)');
title('Velocity distribution in the y direction')
caxis([0,1]);
colorbar;

figure(h3);
set(gcf,'Units','centimeters','Position',[5 5 16 9]);
set(gca,'DataAspectRatio',[1,1,1],'PlotBoxAspectRatio',[1,1,1]...,'xLim',[0,1.2],'yLim',[-0.8,0.8],'zLim',[-0.7,0]...
    ...,'xtick',0.3:0.1:1.2,'ytick',-0.6:0.1:0.6,'ztick',-0.7:0.1:0 ...
    ...,'xgrid','on','ygrid','on','zgrid','on'...
    ,'yDir','reverse');
view(-50,30);
xlabel('X(m)');
ylabel('Z(m)');
zlabel('Y(m)');
title('Velocity distribution in the z direction')
caxis([0,1]);
colorbar;




%% 获取数据，4列分别对应笛卡尔空间的坐标和其速度值
clear all
clc

%预先分配内存
dq0=30;
dq1=40;
dq2=80;
%遍历范围,注意此处的遍历范围应该是关节空间的范围而不是坐标系，故用q表示
%注意H_0x的的坐标，B_0y的坐标的区别，区分于实际机器人的参数
%行程：x推杆0->76mm（坐标系x正方向） y推杆0->-76mm（坐标系y的负方向） R方向正负50°
k1=16*0.0025/26/2/pi;
k2=19/50/28;

%关节空间
abs_max_q0 = -0.076/k1;
abs_max_q1 = -0.076/k1;
abs_max_q2 = 10/k2;
q0 = linspace(0,abs_max_q0,dq0);
q1 = linspace(0,abs_max_q1,dq1);
q2 = linspace(-abs_max_q2,abs_max_q2,dq2);

data = zeros(dq0*dq1*dq2,4);
%Rb是腿在身体坐标系下的旋转矩阵
 Rb = [1,0,0;
     0,1,0;
     0,0,1];

 Vn=2/3*100*pi; %电机额定转速  单位rad/s

 eb_x=[1,0,0]'; %机身坐标系下指定方向（沿x方向）
%  eb=[0,1,0]'; %机身坐标系下指定方向（沿y方向）
%  eb=[0,0,1]'; %机身坐标系下指定方向（沿z方向）
 
n=1;
for i=q0
    for j=q1
        for k=q2
            J=CalJac([i,j,k]);
            Jb=Rb*J; %机身坐标系下的雅可比矩阵
            temp=(inv(Jb))*eb_x;    
            ve=Vn/norm(temp,Inf); 
            data(n,1:3) = FKM([i,j,k]);
            data(n,4) = ve;
            n=n+1
            
        end
    end
end








%% 计算雅可比矩阵
%雅可比矩阵单位用m作为单位计算的
function [J]=CalJac(mot_pos)


q0=mot_pos(1);
q1=mot_pos(2);
q2=mot_pos(3);


%正解
AC = 0.185;
CD = 0.100;
AG = 0.100;
% DE = 0.39224;
DE=0.375;
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
H_0x = -0.020;
B_0y = 0.069;

% H_0x=0.007105;
% B_0y=0.0447558;


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



%%
function [mot_pos3 ]=IKM(ee_position)
% clear all
% clc
%注意：这里的反解结果是没问题的，但是q0,q1的符号与实际使用的相反%
%重新修改 3d版本
% ee_position为末端坐标，此处要求传入末端在腿坐标系xPy的位置，按顺序为xyz
% mot_pos即为驱动杆件的坐标变化值,按顺序为x,y,Rotation
%末端在初始位置的xyz为[256.8 -360.9 0]
%[304.77 -392.91 0]
%竖直转动轴
% ee_position = [347.49,-372.52,0];
% ee_position = [580.72,-329.75,0];


%单位统一用m

theta0 = atan2(ee_position(3),ee_position(1));
q2 = 50 * 28 / 19 * theta0; 
x0=sqrt(ee_position(3)^2+ee_position(1)^2);
y0=ee_position(2);
PA_x=0.048;
PA_y=0.032;

x=x0-PA_x;
y=y0+PA_y;
%末端位置x,y
%固定杆长
AC = 0.185;
CD = 0.100;
AG = 0.100;
% DE = 0.39224;
DE=0.375;
GF = 0.060;
GH = 0.025;
AJ = 0.0905;
LM = 0.020;
BF = 0.120;
%固定角度
%x向初始位置,注意这个初始位置 不是任一点 是H点
%H_0x = 40;
%y向初始位置,即P点比A点在y方向高出的长度
%B_0y = 58;
% H_0x = 7.105;%坐标值
% B_0y = 44.7557;
%在x'Ay'坐标系下的坐标值
H_0x = -0.020;
B_0y = 0.069;



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
Bx=LM;

%两种方法都可以
% GK=abs(Gy-Hy);
% angle_GHK=asin(GK/GH);
% HK=GH*cos(angle_GHK);
% Hx = Gx-HK;

HN = sqrt(GH^2- (Gy-Hy)^2);
Hx = Gx - HN;

deltaX = -(Hx - H_0x);
q0 = 26.0/16.0*deltaX /0.0025*2.0*pi;%X电机

vector_AE = [x y];
vector_CE = CE/AG*vector_AG;
vector_AC = vector_AE-vector_CE;
vector_GF = GF/AC*vector_AC;
vector_AF = vector_AG+vector_GF;

FL = vector_AF(1)-LM;
BL = sqrt(BF^2-FL^2);
By = vector_AF(2)+BL;
deltaY = (By-B_0y);
q1 = 26.0 / 16.0 * deltaY / 0.0025 * 2.0 * pi;
mot_pos3=[q0,q1,q2];

end

