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

% clear all
% clc
% 
% %确定步长 x，z步长
% ds = 0.01;
% %计数器
% n1=1; %x
% n2=1; %y
% n3=1; %z
% 
% %预先分配内存
% dq0=5;
% dq1=6;
% dq2=7;
% %遍历范围,注意此处的遍历范围应该是关节空间的范围而不是坐标系，故用q表示
% %注意H_0x的的坐标，B_0y的坐标的区别，区分于实际机器人的参数
% %行程：x推杆0->76mm（坐标系x正方向） y推杆0->-76mm（坐标系y的负方向） R方向正负50°
% k1=16*0.0025/26/2/pi;
% k2=19/50/28;
% 
% %关节空间
% abs_max_q0 = -0.076/k1;
% abs_max_q1 = -0.076/k1;
% abs_max_q2 = 50/k2;
% q0 = linspace(0,abs_max_q0,dq0);
% q1 = linspace(0,abs_max_q1,dq1);
% q2 = linspace(-abs_max_q2,abs_max_q2,dq2);
% 
% 
% %165.5<x<580.72  -586.68<y<-215.92 -444.86<z<444.86
% %笛卡尔空间,腿坐标系
% x =linspace(0.1655,0.58072,dq0);
% y =linspace(-0.58668,-0.21592,dq1);
% z =linspace(-0.44486,0.44486,dq2);
% 
% %便于切片的计算值
% % abs_max_q0 = -300;
% % abs_max_q1 = -300;
% % abs_max_q2 = 3600;
% 
% 
% 
% [X,Y,Z]=meshgrid(z,x,y); % 这里相当于绘制网格，还需要把数据转换成对应的形式，
% %实际上这里的网格应该是三维空间坐标而不是关节空间
% 
% res = zeros(dq0,dq2,dq1);%储存结果
% V =zeros(dq0,dq2); %储存结果
% 
% %Rb是腿在身体坐标系下的旋转矩阵
%  Rb = [1,0,0;
%      0,1,0;
%      0,0,1];
% 
%  Vn=2/3*100*pi; %电机额定转速  单位rad/s
% 
%  eb=[1,0,0]'; %机身坐标系下指定方向（沿x方向）
% %  eb=[0,1,0]'; %机身坐标系下指定方向（沿y方向）
% %  eb=[0,0,1]'; %机身坐标系下指定方向（沿z方向）
%  
% 
% 
% for i=q1
%     m=1; %横坐标
%     n=1; %纵坐标
%     for j=q0
%         for k=q2
%             J=CalJac([i,j,k]);
%             Jb=Rb*J; %机身坐标系下的雅可比矩阵
%             temp=(inv(Jb))*eb;    
% %             ve(n,1)=Vn/norm(temp,Inf); 
%             ve=Vn/norm(temp,Inf); 
%             V(m,n) = ve;  
%             n=n+1;
%         end
%         n=1;
%         m=m+1;
% 
%     end
%     res(:,:,n3) =V;
%     n3=n3+1
% end

clear all
clc

%确定步长 x，z步长
ds = 0.01;
%计数器
n1=1; %x
n2=1; %y
n3=1; %z

%预先分配内存
dq0=50;
dq1=50;
dq2=50;
num=dq0*dq1*dq2;
%遍历范围,注意此处的遍历范围应该是关节空间的范围而不是坐标系，故用q表示
%注意H_0x的的坐标，B_0y的坐标的区别，区分于实际机器人的参数
%行程：x推杆0->76mm（坐标系x正方向） y推杆0->-76mm（坐标系y的负方向） R方向正负50°
k1=16*0.0025/26/2/pi;
k2=19/50/28;

%关节空间
abs_max_q0 = -0.076/k1;
abs_max_q1 = -0.076/k1;
abs_max_q2 = 50/k2;
q0 = linspace(0,abs_max_q0,dq0);
q1 = linspace(0,abs_max_q1,dq1);
q2 = linspace(-abs_max_q2,abs_max_q2,dq2);





%165.5<x<580.72  -586.68<y<-215.92 -444.86<z<444.86
%笛卡尔空间,腿坐标系
x =linspace(0.1655,0.58072,dq0);
y =linspace(-0.58668,-0.21592,dq1);
z =linspace(-0.44486,0.44486,dq2);

%便于切片的计算值
% abs_max_q0 = -300;
% abs_max_q1 = -300;
% abs_max_q2 = 3600;



[X,Y,Z]=meshgrid(x,y,z); % 这里相当于绘制网格，还需要把数据转换成对应的形式，
%实际上这里的网格应该是三维空间坐标而不是关节空间

res = zeros(dq1,dq0,dq2);%储存结果
V =zeros(dq1,dq0); %储存结果
val = zeros(dq0*dq1*dq2,4);
count =1;
data = zeros(num,4);

%Rb是腿在身体坐标系下的旋转矩阵
 Rb = [1,0,0;
     0,1,0;
     0,0,1];

 Vn=2/3*100*pi; %电机额定转速  单位rad/s

 eb=[1,0,0]'; %机身坐标系下指定方向（沿x方向）
%  eb=[0,1,0]'; %机身坐标系下指定方向（沿y方向）
%  eb=[0,0,1]'; %机身坐标系下指定方向（沿z方向）
 

%如果使用反解解算关节空间的话有个问题是并不是所有的点的组合都可以到达，遍历的空间是立方体，而腿的空间是扇形体，所以此处再试试用运动学正解进行计算

for k=q2
    m=1; %横坐标
    n=1; %纵坐标
    for i=q0
        for j=q1
            J=CalJac([i,j,k]);
            Jb=Rb*J; %机身坐标系下的雅可比矩阵
            temp=(inv(Jb))*eb;    
%             ve(n,1)=Vn/norm(temp,Inf); 
            ve=Vn/norm(temp,Inf); 
            V(m,n) = ve;  
            m=m+1;
            
            val(count,:)=[i,j,k,ve];

            count = count+1;
        end
        m=1;
        n=n+1;
        

    end
    res(:,:,n3) =V;
    n3=n3+1
end

val2 = val(:,4);
writematrix(val,'data.txt')

%% 用等高线绘制看看
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

%直接用关节空间当坐标
[X,Y] = meshgrid(q0,q2);
res = zeros(dq2,dq0);

%Rb是腿在身体坐标系下的旋转矩阵
 Rb = [1,0,0;
     0,1,0;
     0,0,1];

 Vn=2/3*100*pi; %电机额定转速  单位rad/s

 eb=[1,0,0]'; %机身坐标系下指定方向（沿x方向）
%  eb=[0,1,0]'; %机身坐标系下指定方向（沿y方向）
%  eb=[0,0,1]'; %机身坐标系下指定方向（沿z方向）
 
m=1;%横坐标
n=1;%纵坐标
for i=q0
    for j=q2
        J=CalJac([i,-0.38,j]);
        Jb=Rb*J; %机身坐标系下的雅可比矩阵
        temp=(inv(Jb))*eb;    
%             ve(n,1)=Vn/norm(temp,Inf); 
        ve=Vn/norm(temp,Inf); 
        res(m,n)=ve;
        m=m+1;
    end
    n=n+1;
    m=1;
end

%% 绘制等高线图
contourf(X,Y,res,100)
colorbar
xlabel('x');
ylabel('y');






%% plot
%尝试用切片函数去画
xslice = [] ;
yslice = [-0.4];
zslice = [0];
h=figure(1);
slice(X,Y,Z,res,xslice,yslice,zslice)
title('沿x方向的速度最大速度分布')
% xlabel('z');
% ylabel('x');
% zlabel('y');

xlabel('x');
ylabel('y');
zlabel('z');
cb=colorbar;
set(gca,'xdir','reverse');
set(gca,'ydir','reverse'); 

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

 eb=[1,0,0]'; %机身坐标系下指定方向（沿x方向）
%  eb=[0,1,0]'; %机身坐标系下指定方向（沿y方向）
%  eb=[0,0,1]'; %机身坐标系下指定方向（沿z方向）
 
n=1;
for i=q0
    for j=q1
        for k=q2
            J=CalJac([i,j,k]);
            Jb=Rb*J; %机身坐标系下的雅可比矩阵
            temp=(inv(Jb))*eb;    
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



%% 计算正解，传入关节空间，计算笛卡尔空间坐标,试试看进行向量化

function [ee_position]=FKM(mot_pos)
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


%单位为m
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

PA_x=0.048;
PA_y=0.032;
% mot_pos=[-318.5526,310.3820,-64.2526]; %电机输入量，目前是二维，分别为xy方向电机输入量




deltaX=-16*0.0025/26/2/pi*mot_pos(1);
deltaY=16*0.0025/26/2/pi*mot_pos(2);
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

end
