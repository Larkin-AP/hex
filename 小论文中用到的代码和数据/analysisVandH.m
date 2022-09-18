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



%% P1 按切片方式计算函数
%想办法把res转换成网格
%{
以q0，q1，q2的元素个数分别为21，6，11的情况下进行说明
我们固定y，网格平面为x*z的平面
即相当于有6张纸，每张纸为21*11的网格，每个网格对应其中的值
res应该是21*
%}


clear all
clc

[x_list,z_list] = meshgrid(0.1065:0.005:0.5771,-0.4419:0.005:0.4419);
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

Vmax = 0;%用于记录三个方向中，整个空间的最大速度


for y=-0.5867:0.01:-0.2199
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
            [q,judge]=IKM([x_list(i,j),y,z_list(i,j)]); %q是向量
            if isreal(q) %q为实数，说明该点为工作空间，进入循环
                if judge ==true
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
    end

    
    figure(h1);
    hold on;
    surf(x_list,z_list,y_list,vel_list_x,'EdgeColor','none');  
    Vmax_x = max(max(vel_list_x));
    
    figure(h2);
    hold on;
    surf(x_list,z_list,y_list,vel_list_y,'EdgeColor','none');    
    Vmax_y = max(max(vel_list_y));
    
    figure(h3);
    hold on;
    surf(x_list,z_list,y_list,vel_list_z,'EdgeColor','none');
    Vmax_z = max(max(vel_list_z));

    temp_v_max = max([Vmax_x,Vmax_y,Vmax_z]);
    if Vmax<temp_v_max
        Vmax=temp_v_max;
    end
    
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
ax1 =gca;
caxis([0,1]);
colorbar;
colormap turbo;

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
colormap turbo;
ax2 = gca;

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
colormap turbo;
ax3 = gca;




figure %整图的信息

fnew = figure;
ax1_copy = copyobj([ax1],fnew);
hh1=subplot(2,2,2,ax1_copy); %子图
pose1 = set(hh1,'Units','centimeters','Position',[14.5 8 14 6.5]); %设置子图位置
set(gca,'xticklabel','');
set(gca,'yticklabel','');
set(gca,'zticklabel','');
xlabel('');
zlabel('');
ylabel('');
zlim([-0.7,-0]);
ylim([-0.45,0.45]);
xlim([0,0.7]);
set(gca,'ztick',-0.8:0.2:0);
set(gca,'xtick',0:0.2:0.7);
title('Along x-direction','Units','centimeters','Position',[1.5,5]);
set(gca,'FontName','Times New Roman','FontSize',20);
colormap turbo;
grid on


copies = copyobj(ax2,fnew);
ax2_copy = copies(1);
hh2=subplot(2,2,4,ax2_copy); %子图
pose2 = set(hh2,'Units','centimeters','Position',[14.5 1 14 6.5]); %设置子图位置
set(gca,'xticklabel','');
set(gca,'yticklabel','');
set(gca,'zticklabel','');
xlabel('');
zlabel('');
ylabel('');
zlim([-0.7,-0]);
ylim([-0.45,0.45]);
xlim([0,0.7]);
set(gca,'ztick',-0.8:0.2:0);
set(gca,'xtick',0:0.2:0.7);
title('Along y-direction','Units','centimeters','Position',[1.5,5]);
set(gca,'FontName','Times New Roman','FontSize',20);
colormap turbo;
grid on

copies = copyobj(ax3,fnew);
ax3_copy = copies(1);
hh3=subplot(2,2,[1,3],ax3_copy); %子图
pose3 = set(hh3,'Units','centimeters','Position',[2 2 15 11]); %设置子图位置
zlim([-0.7,-0]);
ylim([-0.45,0.45]);
xlim([0,0.7]);
set(gca,'ztick',-0.8:0.2:0);
set(gca,'xtick',0:0.2:0.7);
set(gca,'FontName','Times New Roman','FontSize',20);
title('Along z-direction','Units','centimeters','Position',[2,10]);

colormap turbo;

set(gcf,'Units','centimeters','Position',[5 5 30 15]);
grid on
cb=colorbar;
pose4 = set(cb,'Units','centimeters','Position',[26.5 1.5 1 12]); %设置色棒位置

caxis([0,Vmax]);%设置颜色数据最大值
t1 = caxis;
t1 = linspace(t1(1),t1(2),6);
set(cb,'ytick',t1);



%% P2 寻找速度分布中好的空间
%目标1、先把每层好的区域画出来


%想办法把res转换成网格
%{
以q0，q1，q2的元素个数分别为21，6，11的情况下进行说明
我们固定y，网格平面为x*z的平面
即相当于有6张纸，每张纸为21*11的网格，每个网格对应其中的值
res应该是21*
%}


clear all
clc

[x_list,z_list] = meshgrid(0.1065:0.01:0.5771,-0.4419:0.01:0.4419);
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

%记录每层切面的速度最大值
count = 1;
s = max(size(-0.5867:0.05:-0.2199));
max_x_list = nan(1,s);
max_y_list = nan(1,s);
max_z_list = nan(1,s);


for y=-0.5867:0.05:-0.2199
    y_list=ones(m,n)*y;
    vel_list_x =nan(m,n);
    vel_list_y =nan(m,n);
    vel_list_z =nan(m,n);

    %找到当前切面内的速度最大值
    a=0.5; %目前拟定区间为[a*max,max],a为系数

    for i=1:m
        for j=1:n
            %[x_list(i,j),y,z_list(i,j)]取出来该点
            %先判断该点是否在工作空间内
            %用运动学反解，如果是复数，直接跳出本次循环
            %如果是实数，继续内容
            %运动学反解求解出关节空间坐标
            %根据关节空间求解雅可比矩阵和最大速度
            %把速度赋值给vel_list即可完成本次循环
            [q,judge]=IKM([x_list(i,j),y,z_list(i,j)]); %q是向量
            if isreal(q) %q为实数，说明该点为工作空间，进入循环
                if judge == true 
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
    end
    max_x = max(max(vel_list_x));
    max_y = max(max(vel_list_y));
    max_z = max(max(vel_list_z));
    
    max_x_list(count) = max_x;
    max_y_list(count) = max_y;
    max_z_list(count) = max_z; 
    count =count+1;

    
    %筛选出速度分布满足[a*max,max]的空间点集
    for i=1:m
        for j=1:n
            if vel_list_x(i,j)<a*max_x
                vel_list_x(i,j) =nan;
            end
            if vel_list_y(i,j)<a*max_y
                vel_list_y(i,j) =nan;
            end
            if vel_list_z(i,j)<a*max_z
                vel_list_z(i,j) =nan;
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
colormap turbo;

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
colormap turbo;

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
colormap turbo;



%% P3 寻找速度分布中好的空间2
%此处与上面的方法略有不同，此处的最大值为整个空间的最大值

clear all
clc

[x_list,z_list] = meshgrid(0.1065:0.01:0.5771,-0.4419:0.01:0.4419);
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

%记录每层切面的速度最大值
count = 1;
s = max(size(-0.5867:0.01:-0.2199));
max_x_list = nan(1,s);
max_y_list = nan(1,s);
max_z_list = nan(1,s);


for y=-0.5867:0.01:-0.2199
    y_list=ones(m,n)*y;
    vel_list_x =nan(m,n);
    vel_list_y =nan(m,n);
    vel_list_z =nan(m,n);

    %找到当前切面内的速度最大值
    a=0.5; %目前拟定区间为[a*max,max],a为系数

    for i=1:m
        for j=1:n
            %[x_list(i,j),y,z_list(i,j)]取出来该点
            %先判断该点是否在工作空间内
            %用运动学反解，如果是复数，直接跳出本次循环
            %如果是实数，继续内容
            %运动学反解求解出关节空间坐标
            %根据关节空间求解雅可比矩阵和最大速度
            %把速度赋值给vel_list即可完成本次循环
            [q,judge]=IKM([x_list(i,j),y,z_list(i,j)]); %q是向量
            if isreal(q) %q为实数，说明该点为工作空间，进入循环
                if judge == true
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
    end
    max_x = max(max(vel_list_x));
    max_y = max(max(vel_list_y));
    max_z = max(max(vel_list_z));
    
    max_x_list(count) = max_x;
    max_y_list(count) = max_y;
    max_z_list(count) = max_z; 
    count =count+1;
    
end

max_x_all = max(max_x_list);
max_y_all = max(max_y_list);
max_z_all = max(max_z_list);



for y=-0.5867:0.01:-0.2199
    y_list=ones(m,n)*y;
    vel_list_x =nan(m,n);
    vel_list_y =nan(m,n);
    vel_list_z =nan(m,n);

    %找到当前切面内的速度最大值
    a=0.5; %目前拟定区间为[a*max,max],a为系数
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
        
            
    %筛选出速度分布满足[a*max,max]的空间点集
    for i=1:m
        for j=1:n
            if vel_list_x(i,j)<a*max_x_all
                vel_list_x(i,j) =nan;
            end
            if vel_list_y(i,j)<a*max_y_all
                vel_list_y(i,j) =nan;
            end
            if vel_list_z(i,j)<a*max_z_all
                vel_list_z(i,j) =nan;
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
colormap turbo;

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
colormap turbo;

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
colormap turbo;

%% P4 以切片绘制为单位绘制图
%目标1、先把每层好的区域画出来


%想办法把res转换成网格
%{
以q0，q1，q2的元素个数分别为21，6，11的情况下进行说明
我们固定y，网格平面为x*z的平面
即相当于有6张纸，每张纸为21*11的网格，每个网格对应其中的值
res应该是21*
%}


clear all
clc

[x_list,z_list] = meshgrid(0.1065:0.01:0.5771,-0.4419:0.01:0.4419);
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

%记录每层切面的速度最大值
count = 1;
s = max(size(-0.5867:0.05:-0.2199));
max_x_list = nan(1,s);
max_y_list = nan(1,s);
max_z_list = nan(1,s);


for y=-0.5867:0.05:-0.2199
    y_list=ones(m,n)*y;
    vel_list_x =nan(m,n);
    vel_list_y =nan(m,n);
    vel_list_z =nan(m,n);

    %找到当前切面内的速度最大值
    a=0.5; %目前拟定区间为[a*max,max],a为系数

    for i=1:m
        for j=1:n
            %[x_list(i,j),y,z_list(i,j)]取出来该点
            %先判断该点是否在工作空间内
            %用运动学反解，如果是复数，直接跳出本次循环
            %如果是实数，继续内容
            %运动学反解求解出关节空间坐标
            %根据关节空间求解雅可比矩阵和最大速度
            %把速度赋值给vel_list即可完成本次循环
            [q,judge]=IKM([x_list(i,j),y,z_list(i,j)]); %q是向量
            if isreal(q) %q为实数，说明该点为工作空间，进入循环
                if judge == true
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
    end
    max_x = max(max(vel_list_x));
    max_y = max(max(vel_list_y));
    max_z = max(max(vel_list_z));
    
    max_x_list(count) = max_x;
    max_y_list(count) = max_y;
    max_z_list(count) = max_z; 
    

    
    %筛选出速度分布满足[a*max,max]的空间点集
    for i=1:m
        for j=1:n
            if vel_list_x(i,j)<a*max_x
                vel_list_x(i,j) =nan;
            end
            if vel_list_y(i,j)<a*max_y
                vel_list_y(i,j) =nan;
            end
            if vel_list_z(i,j)<a*max_z
                vel_list_z(i,j) =nan;
            end
        end
    end
    
    
    figure(h1);
    subplot(2,4,count);
    surf(x_list,z_list,y_list,vel_list_x,'EdgeColor','none'); 
    subtitle(['y= ',num2str(y)])
    view(2);
    xlabel('X(m)');
    ylabel('Z(m)');
    zlabel('Y(m)');
    caxis([0,1]);
    colorbar;
    colormap turbo;

    
    figure(h2);
    subplot(2,4,count);
    surf(x_list,z_list,y_list,vel_list_z,'EdgeColor','none'); 
    subtitle(['y= ',num2str(y)])
    view(2);
    xlabel('X(m)');
    ylabel('Z(m)');
    zlabel('Y(m)');
    caxis([0,1]);
    colorbar;
    colormap turbo;
    
    
    count =count+1;
    
    
end

figure(h1);
sgtitle('Velocity distribution in the x direction');
set(gcf,'Units','centimeters','Position',[5 5 32 18]); %指定plot输出图片的尺寸，xmin，ymin，width，height

figure(h2);
sgtitle('Velocity distribution in the z direction');
set(gcf,'Units','centimeters','Position',[5 5 32 18]); %指定plot输出图片的尺寸，xmin，ymin，width，height

%% P5 分层画交集
clear all
clc

[x_list,z_list] = meshgrid(0.1065:0.01:0.5771,-0.4419:0.01:0.4419);
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


%记录每层切面的速度最大值
count = 1;
s = max(size(-0.5867:0.05:-0.2199));
max_x_list = nan(1,s);
max_y_list = nan(1,s);
max_z_list = nan(1,s);


for y=-0.5867:0.05:-0.2199
    y_list=ones(m,n)*y;
    vel_list_x =nan(m,n);
    vel_list_y =nan(m,n);
    vel_list_z =nan(m,n);
    intersection = nan(m,n);

    %找到当前切面内的速度最大值
    a=0.5; %目前拟定区间为[a*max,max],a为系数

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
    max_x = max(max(vel_list_x));
    max_y = max(max(vel_list_y));
    max_z = max(max(vel_list_z));
    
    max_x_list(count) = max_x;
    max_y_list(count) = max_y;
    max_z_list(count) = max_z; 
    

    
    %筛选出速度分布满足[a*max,max]的空间点集
    for i=1:m
        for j=1:n
            if vel_list_x(i,j)<a*max_x
                vel_list_x(i,j) =nan;
            end
            if vel_list_y(i,j)<a*max_y
                vel_list_y(i,j) =nan;
            end
            if vel_list_z(i,j)<a*max_z
                vel_list_z(i,j) =nan;
            end
            if ~(isnan(vel_list_x(i,j)) || isnan(vel_list_y(i,j)) || isnan(vel_list_z(i,j)))
                intersection(i,j) = 1; 
            end%这个地方交集应该没问题
        end
    end
    
    
    figure(h1);
    subplot(2,4,count);
    surf(x_list,z_list,y_list,intersection,'EdgeColor','none'); 
    subtitle(['y= ',num2str(y)])
    view(2);
    xlabel('X(m)');
    ylabel('Z(m)');
    zlabel('Y(m)');
    caxis([0,1]);
    colorbar;
    colormap turbo;

    

    
    
    count =count+1;
    
    
end

figure(h1);
sgtitle('Velocity distribution in the x direction');
set(gcf,'Units','centimeters','Position',[5 5 32 18]); %指定plot输出图片的尺寸，xmin，ymin，width，height








%% P6 寻找交集
%目标2、先把每层交集画出来
%先求空间交集，再画每层的速度分布


%想办法把res转换成网格
%{
以q0，q1，q2的元素个数分别为21，6，11的情况下进行说明
我们固定y，网格平面为x*z的平面
即相当于有6张纸，每张纸为21*11的网格，每个网格对应其中的值
res应该是21*
%}







clear all
clc

[x_list,z_list] = meshgrid(0.1065:0.01:0.5771,-0.4419:0.01:0.4419);
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
h4=figure;


%记录每层切面的速度最大值
count = 1;
s = max(size(-0.5867:0.05:-0.2199));
max_x_list = nan(1,s);
max_y_list = nan(1,s);
max_z_list = nan(1,s);


for y=-0.5867:0.05:-0.2199
    y_list=ones(m,n)*y;
    vel_list_x =nan(m,n);
    vel_list_y =nan(m,n);
    vel_list_z =nan(m,n);
    intersection = nan(m,n);%交集，函数值以z的值做表示，

    %找到当前切面内的速度最大值
    a=0.5; %目前拟定区间为[a*max,max],a为系数

    for i=1:m
        for j=1:n
            %[x_list(i,j),y,z_list(i,j)]取出来该点
            %先判断该点是否在工作空间内
            %用运动学反解，如果是复数，直接跳出本次循环
            %如果是实数，继续内容
            %运动学反解求解出关节空间坐标
            %根据关节空间求解雅可比矩阵和最大速度
            %把速度赋值给vel_list即可完成本次循环
            [q,judge]=IKM([x_list(i,j),y,z_list(i,j)]); %q是向量
            if isreal(q) %q为实数，说明该点为工作空间，进入循环
                if judge == true
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
    end
    max_x = max(max(vel_list_x));
    max_y = max(max(vel_list_y));
    max_z = max(max(vel_list_z));
    
    max_x_list(count) = max_x;
    max_y_list(count) = max_y;
    max_z_list(count) = max_z; 
    
    
    %先筛选出速度分布满足[a*max,max]的空间点集
    %再求空间交集
    for i=1:m
        for j=1:n
            if vel_list_x(i,j)<a*max_x
                vel_list_x(i,j) =nan;
            end
            if vel_list_y(i,j)<a*max_y
                vel_list_y(i,j) =nan;
            end
            if vel_list_z(i,j)<a*max_z
                vel_list_z(i,j) =nan;
            end
            if ~(isnan(vel_list_x(i,j)) || isnan(vel_list_y(i,j)) || isnan(vel_list_z(i,j)))
                intersection(i,j) = 1; 
            end%这个地方交集应该没问题
            %再逐个画出交集对应的速度分布
            if ~isnan(vel_list_x(i,j)) && isnan(intersection(i,j))
                vel_list_x(i,j)=nan;
            end
            if ~isnan(vel_list_y(i,j)) && isnan(intersection(i,j))
                vel_list_y(i,j)=nan;
            end
            if ~isnan(vel_list_z(i,j)) && isnan(intersection(i,j))
                vel_list_z(i,j)=nan;
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
    
    figure(h4);
    subplot(2,4,count);
    surf(x_list,z_list,y_list,intersection,'EdgeColor','none'); 
    subtitle(['y= ',num2str(y)])
    view(2);
    xlabel('X(m)');
    ylabel('Z(m)');
    zlabel('Y(m)');
    caxis([0,1]);
    colorbar;
    colormap turbo;
        
    
    
    count =count+1;
 
    
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
colormap turbo;

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
colormap turbo;

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
colormap turbo;



figure(h4);
sgtitle('Intersection space');
set(gcf,'Units','centimeters','Position',[5 5 32 18]); %指定plot输出图片的尺寸，xmin，ymin，width，height

%% P7 尝试求一下极值
%F = alpha(H/Hmax)+beta(V/Vmax)
clear all
clc

alpha =0.5;
beta = 1-alpha;
Vmax = 1.6968;%整个空间的z向速度最大值，间隔为0.01
y0=0.11;
% H=y-y0;%涉水高度H
ymin = -0.5867;  %ymin是腿能达到的最远空间
Hmax = -ymin-y0;

[x_list,z_list] = meshgrid(0.1065:0.01:0.580,-0.4419:0.01:0.4419);

[m,n]=size(x_list);
%Rb是腿在身体坐标系下的旋转矩阵
 Rb = [1,0,0;
     0,1,0;
     0,0,1];

 Vn=2/3*100*pi; %电机额定转速  单位rad/s

%  eb_x=[1,0,0]'; %机身坐标系下指定方向（沿x方向）
%  eb_y=[0,1,0]'; %机身坐标系下指定方向（沿y方向）
 eb_z=[0,0,1]'; %机身坐标系下指定方向（沿z方向）

h1=figure;
h2=figure;
h3=figure;
F_max_list = nan(74,4);%每一行对应每一层，四列分别该层出现最大值的对应x，y，z坐标以及F函数值
%记录每层极值出现的时候对应的坐标
count =1;




for y=-0.5867:0.005:-0.2199
    y_list=ones(m,n)*y;
    F_list =nan(m,n);
    ratio_list = nan(m,n);

    for i=1:m
        for j=1:n
            %[x_list(i,j),y,z_list(i,j)]取出来该点
            %先判断该点是否在工作空间内
            %用运动学反解，如果是复数，直接跳出本次循环
            %如果是实数，继续内容
            %运动学反解求解出关节空间坐标
            %根据关节空间求解雅可比矩阵和最大速度
            %把速度赋值给vel_list即可完成本次循环
            [q,judge]=IKM([x_list(i,j),y,0]); %q是向量
    
            if isreal(q) %q为实数，说明该点为工作空间，进入循环
                if judge==true
                    
                    J=CalJac(q);
                    Jb=Rb*J;
                    temp_z=(inv(Jb))*eb_z;
                    ve_z=Vn/norm(temp_z,Inf);
                    ratio = ve_z/Vmax; 
                    ratio_list(i,j) =ratio;  
                    F = alpha*((-y-y0)/Hmax)+beta*ratio;
                    F_list(i,j) =F; 

                end

            end               
        end
    end

    v_max = max(max(F_list));
    F_max_list(count,4)=v_max;
    F_max_list(count,2)=y;
    if isnan(v_max)
        F_max_list(count,1)=nan;
        F_max_list(count,3)=nan;
    else
        [row, col] = find(F_list == v_max);
        row = row(1);
        col = col(1);
        F_max_list(count,1)=x_list(row,col);
        F_max_list(count,3)=z_list(row,col);
    end

    figure(h1);
    hold on;
    surf(x_list,z_list,y_list,F_list,'EdgeColor','none');  
    count =count+1;
    

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
title('F distribution in the z direction')
caxis([0,1]);
colorbar;
colormap turbo;

figure(h2);
y=-0.5867:0.005:-0.2199;
F_max_val =F_max_list(:,4)';
plot(y,F_max_val);
[kmax,id] = findkmax(F_max_val,3);
xlabel('末端y坐标');
ylabel('F函数值');
title('F随末端y坐标的变化')
% [val ,pos] = max(F_max_val);
% text(y(pos),F_max_val(pos),['极值点(',num2str(y(pos)),',',num2str(F_max_val(pos)),')']);
for i=1:3
    text(y(id(i)),F_max_val(id(i))-(i-1)*0.006,['极值点',num2str(i),'(',num2str(y(id(i))),',',num2str(F_max_val(id(i))),')'])
    hold on
    plot(y(id(i)),F_max_val(id(i)),'-*');
end


figure(h3);
scatter3(F_max_list(:,1),F_max_list(:,3),F_max_list(:,2));
set(gcf,'Units','centimeters','Position',[5 5 16 9]); %指定plot输出图片的尺寸，xmin，ymin，width，height
set(gca,'DataAspectRatio',[1,1,1],'PlotBoxAspectRatio',[1,1,1]...,'xLim',[0,1.2],'yLim',[-0.8,0.8],'zLim',[-0.7,0]...
    ...,'xtick',0.3:0.1:1.2,'ytick',-0.6:0.1:0.6,'ztick',-0.7:0.1:0 ...
    ...,'xgrid','on','ygrid','on','zgrid','on'...
    ,'yDir','reverse');
xlabel('X(m)');
ylabel('Z(m)');
zlabel('Y(m)');
title('每层最大函数值出现的足端坐标点')
view(-50,30);

h4 = copy(h3);
figure(h4);
view(0,90);



%% P8 看一下alpha对函数值大小的影响
%F = alpha(H/Hmax)+beta(V/Vmax)
clear all
clc

Vmax = 1.6157;%整个空间的z向速度最大值，这个vmax和y的取值间隔有关系，注意保持一致
y0=0.11;
% H=y-y0;%涉水高度H
ymin = -0.5476;  %ymin是腿能达到的最远空间
Hmax = -ymin-y0;

[x_list,z_list] = meshgrid(0.1065:0.01:0.5771,-0.4419:0.01:0.4419);
[m,n]=size(x_list);
%Rb是腿在身体坐标系下的旋转矩阵
 Rb = [1,0,0;
     0,1,0;
     0,0,1];

 Vn=2/3*100*pi; %电机额定转速  单位rad/s

%  eb_x=[1,0,0]'; %机身坐标系下指定方向（沿x方向）
%  eb_y=[0,1,0]'; %机身坐标系下指定方向（沿y方向）
 eb_z=[0,0,1]'; %机身坐标系下指定方向（沿z方向）

h1=figure;

F_max_list = nan(37,11);
row = 1;
col = 1;

for alpha = 0:0.1:1
    beta = 1-alpha;
    
    for y=-0.5867:0.01:-0.2199
        y_list=ones(m,n)*y;
        ratio_list = nan(m,n);
        F_list = nan(m,n);
        for i=1:m
            for j=1:n
                [q,judge]=IKM([x_list(i,j),y,z_list(i,j)]); %q是向量
                if isreal(q) %q为实数，说明该点为工作空间，进入循环
                    if judge == true
                        J=CalJac(q);
                        Jb=Rb*J;
                        temp_z=(inv(Jb))*eb_z;
                        ve_z=Vn/norm(temp_z,Inf);
                        ratio = ve_z/Vmax; 
                        ratio_list(i,j) =ratio;  
                        F = alpha*((-y-y0)/Hmax)+beta*ratio;
                        F_list(i,j) =F;  
                    end
                end             
            end               
        end
        v_max = max(max(F_list)); %每一层切片的最大函数值
        F_max_list(row,col) = v_max;
        row = row+1;
        

    end
    figure(h1);
    plot(-0.5867:0.01:-0.2199,F_max_list(:,col));
    hold on
    col=col+1
    row=1;
    
    
             
end





figure(h1);
set(gcf,'Units','centimeters','Position',[5 5 25 17]); %指定plot输出图片的尺寸，xmin，ymin，width，height
% title('The effect of different \alpha on the function value F','FontSize',30,'FontWeight','bold');

col=1;
% axis([-0.55,-0.18,0.2,1.05])
% title('');
grid off;
set(gca,'FontName','Times New Roman','FontSize',20);
xlim([-0.58,-0.2]);
ylim([0.2 1.1]);
set(gca,'ytick',[0.2:0.2:1]);
xlabel('y(m)','FontSize',24);
ylabel('F','FontSize',24);




%交点
%第一条线
point1 = [-0.4567,0.792276];
point2 = [-0.4467,0.769424];
%第二条线
point3 = [-0.4567,0.778761];
point4 = [-0.4467,0.800594];
[insec_node(1),insec_node(2)] = node(point1,point2,point3,point4)
insec_node = round(insec_node,3);

plot(insec_node(1),insec_node(2),'*','MarkerSize',8,'Color','r','LineWidth',1.2);
text(insec_node(1)-0.005,insec_node(2)+0.03,['P (',num2str(insec_node(1)),',',num2str(insec_node(2)),')'],'FontSize',20,'FontWeight','bold','Color','black');
for i=1:11
    
    alpha =0:0.1:1;
    leg_str{i} = ['\alpha=',num2str(alpha(i))];
%     text(-0.585,F_max_list(1,col),['alpha=',num2str(alpha(i))]);
    col=col+1;
end
lgd = legend(leg_str,'FontSize',16,'TextColor','black','NumColumns',3,'Location','southwest');
% legend(leg_str);
hold on





%% P9 求极值平面
%F = alpha(H/Hmax)+beta(V/Vmax)
clear all
clc

alpha =0.5;
beta = 1-alpha;
Vmax = 1.6968;%整个空间的z向速度最大值
y0=0.11;
% H=y-y0;%涉水高度H
ymin = -0.5867;  %ymin是腿能达到的最远空间
Hmax = -ymin-y0;

[x_list,z_list] = meshgrid(0.1065:0.001:0.5771,-0.4419:0.001:0.4419);
[m,n]=size(x_list);
%Rb是腿在身体坐标系下的旋转矩阵
 Rb = [1,0,0;
     0,1,0;
     0,0,1];

 Vn=2/3*100*pi; %电机额定转速  单位rad/s

%  eb_x=[1,0,0]'; %机身坐标系下指定方向（沿x方向）
%  eb_y=[0,1,0]'; %机身坐标系下指定方向（沿y方向）
 eb_z=[0,0,1]'; %机身坐标系下指定方向（沿z方向）

h1=figure;





y=-0.4667;
y_list=ones(m,n)*y;
F_list =nan(m,n);
ratio_list = nan(m,n);

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
            temp_z=(inv(Jb))*eb_z;
            ve_z=Vn/norm(temp_z,Inf);
            ratio = ve_z/Vmax; 
            ratio_list(i,j) =ratio;  
            F = alpha*((-y-y0)/Hmax)+beta*ratio;
            F_list(i,j) =F;  

        end               
    end
end


figure(h1);
hold on
surf(x_list,z_list,y_list,F_list,'EdgeColor','none');  
xlabel('X(m)');
ylabel('Z(m)');
zlabel('Y(m)');
caxis([0,1]);
colorbar;
colormap turbo;
hold on

F_max =max(max(F_list));
F_list2 =nan(m,n);
[row, col]=find(F_list==F_max);
x_coord = x_list(row,col);
z_coord = z_list(row,col);
plot(x_coord,z_coord,'-*k');

h2=figure; %保留[amax,max]范围内的值的平面

a=0.5;
for i =1:m
    for j = 1:n
        if F_list(i,j) >=a*F_max
            F_list2(i,j) = F_list(i,j);
        end
    end
end

figure(h2);
hold on;
surf(x_list,z_list,y_list,F_list2,'EdgeColor','none');  
xlabel('X(m)');
ylabel('Z(m)');
zlabel('Y(m)');
title(['a = ' , num2str(a)]);
caxis([0,1]);
colorbar;
colormap turbo;
hold on

%% P10 看下三维空间中可操作度椭球体积的分布
clear all
clc

[x_list,z_list] = meshgrid(0.1065:0.005:0.5771,-0.4419:0.005:0.4419);
[m,n]=size(x_list);

h1=figure;
h2=figure;
count =1;

for y=-0.5867:0.025:-0.2199
    y_list=ones(m,n)*y;
    V_ellipsoid_list =nan(m,n);


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
                A=J*J';
                [V,D] = eig(A);
                D = diag(D);
                V_ellipsoid = sqrt(det(A)); %椭球体积正比于该值，并不完全代表椭球体积

                V_ellipsoid_list(i,j) =V_ellipsoid*10^7.5;    
            end               
        end
    end

    
    figure(h1);
    hold on;
    surf(x_list,z_list,y_list,V_ellipsoid_list,'EdgeColor','none');  
    
    figure(h2);
    subplot(4,4,count);
    surf(x_list,z_list,y_list,V_ellipsoid_list,'EdgeColor','none');  
    subtitle(['y= ',num2str(y)])
    view(2);
    xlabel('X(m)');
    ylabel('Z(m)');
    zlabel('Y(m)');
    caxis([0,1]);
    colorbar;
    colormap turbo;
    
    
    count =count +1;
    
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
title('可操作度椭球体积在三维空间中的分布*10^7')
caxis([0,1]);
colorbar;
% colormap turbo;

%% P11 让z坐标都为0，去计算F
%F = alpha(H/Hmax)+beta(V/Vmax)
clear all
clc

alpha =0.5;
beta = 1-alpha;
%缩短20%前
% Vmax = 1.6157;%整个空间的z向速度最大值，间隔为0.005
% ymin = -0.5467;  %ymin是腿能达到的最远空间
%缩短20%后
Vmax = 1.5874;%整个空间的z向速度最大值，间隔为0.005
ymin = -0.5067;  %ymin是腿能达到的最远空间
y0=0.11;
% H=y-y0;%涉水高度H

Hmax = -ymin-y0;

[x_list,y_list] = meshgrid(0.1065:0.001:0.580,-0.5867:0.001:-0.2199);

[m,n]=size(x_list);
%Rb是腿在身体坐标系下的旋转矩阵
 Rb = [1,0,0;
     0,1,0;
     0,0,1];

 Vn=2/3*100*pi; %电机额定转速  单位rad/s

%  eb_x=[1,0,0]'; %机身坐标系下指定方向（沿x方向）
%  eb_y=[0,1,0]'; %机身坐标系下指定方向（沿y方向）
 eb_z=[0,0,1]'; %机身坐标系下指定方向（沿z方向）

h1=figure;
% h2=figure;
% h3=figure;
F_max_list = nan(m,6);%每一行对应每一层，四列分别该层出现最大值的对应x，y，z坐标以及,H,V,F函数值
F_max_list(:,3)=0;
%记录每层极值出现的时候对应的坐标



count =1;

F_list =nan(m,n);
V_list =nan(m,n);
H_list =nan(m,n);

ratio_list = nan(m,n);

for i=1:m
    for j=1:n
        %[x_list(i,j),y,z_list(i,j)]取出来该点
        %先判断该点是否在工作空间内
        %用运动学反解，如果是复数，直接跳出本次循环
        %如果是实数，继续内容
        %运动学反解求解出关节空间坐标
        %根据关节空间求解雅可比矩阵和最大速度
        %把速度赋值给vel_list即可完成本次循环
        [q,judge]=IKM([x_list(i,j),y_list(i,j),0]); %q是向量

        if isreal(q) %q为实数，说明该点为工作空间，进入循环
            if judge==true

                J=CalJac(q);
                Jb=Rb*J;
                temp_z=(inv(Jb))*eb_z;
                ve_z=Vn/norm(temp_z,Inf);
                ratio = ve_z/Vmax; 
                ratio_list(i,j) =ratio;  
                F = alpha*((-y_list(i,j)-y0)/Hmax)+beta*ratio;
                F_list(i,j) =F; 
                V_list(i,j) =ve_z;
                H_list(i,j) =(-y_list(i,j)-y0);

            end
        end               
    end
end

[max_a,index]=max(F_list,[],2); %每一行最大值以及最大值的列位置
F_max_list(:,6) = max_a; %F_max_list 每一列分别对应 x y z 和H,V,F值
index_size = max(size(index));

H_list_col = nan(index_size,1);
V_list_col = nan(index_size,1);
sum_list_col = nan(index_size,1);

for i=1:index_size
   H_list_col(i,1) = H_list(i,index(i));
   V_list_col(i,1) = V_list(i,index(i));
   sum_list_col(i,1) = 0.5*V_list_col(i,1)/Vmax+0.5*H_list_col(i,1)/Hmax;
end

F_max_list(:,4) = H_list_col;
F_max_list(:,5) = V_list_col;

% F_max_list(:,4) = H_list(:,index');
% F_max_list(:,5) = V_list(:,index');
F_max_list(:,2) = y_list(:,1);
for i =1:m
    if isnan(F_max_list(i,6))
        F_max_list(i,:) = nan;
    else
        F_max_list(i,1) = x_list(i,index(i));
    end
    
end
[Fmax,row]=max(F_max_list(:,6),[],1);
Fmax_x = F_max_list(row,1)
Fmax_y = F_max_list(row,2)
Fmax_h = F_max_list(row,4)
Fmax_v = F_max_list(row,5)
Fmax_f = F_max_list(row,6)

figure(h1);
hold on;
surf(x_list,y_list,F_list,'EdgeColor','none');  
% surf(x_list,y_list,F_list);  

    


xaxis = [0.1,0.7];
yaxis = [-0.6,-0.2];
figure(h1);
set(gcf,'Units','centimeters','Position',[5 5 25 14]); %指定plot输出图片的尺寸，xmin，ymin，width，height
% set(gca,'DataAspectRatio',[1,1,1],'PlotBoxAspectRatio',[1,1,1]...,'xLim',[0,1.2],'yLim',[-0.8,0.8],'zLim',[-0.7,0]...
%     ...,'xtick',0.3:0.1:1.2,'ytick',-0.6:0.1:0.6,'ztick',-0.7:0.1:0 ...
%     ...,'xgrid','on','ygrid','on','zgrid','on'...
%     );
% title('F distribution in the z direction(z=0)','FontSize',20,'FontWeight','bold');
set(gca,'FontName','Times New Roman','FontSize',20);
xlabel('x(m)','FontSize',24);
ylabel('y(m)','FontSize',24);
xlim(xaxis);
ylim(yaxis);
set(gca,'ytick',[-0.55:0.1:-0.25]);
caxis([0,1]);
cb=colorbar;

colormap turbo;
hold on
scatter(F_max_list(:,1),F_max_list(:,2),[],'blue');
hold on
plot(Fmax_x,Fmax_y,'*','MarkerSize',12,'Color','r','LineWidth',2)
hold on
% Fmax_x = round(Fmax_x,3);
% Fmax_y = round(Fmax_y,3);
% text(Fmax_x,Fmax_y,['\leftarrow F_{max} (',num2str(Fmax_x),',',num2str(Fmax_y),')'],'FontSize',24)
% Fmax_x
% Fmax_y

Fmax_ele = max(max(F_list));
caxis([0,Fmax_ele]);%设置颜色数据最大值
t1 = caxis;
t1 = linspace(t1(1),t1(2),6);
t1 = [0,0.1564,0.3128,0.4693,0.6257,0.7821];
set(cb,'ytick',t1);
% set(cb,'yticklabel','');


% rel_x = (Fmax_x-xaxis(1))/(xaxis(2)-xaxis(1))
% rel_y = (Fmax_y-yaxis(1))/(yaxis(2)-yaxis(1))
% x = [rel_x+0.2 rel_x];
% y = [rel_y-0.1 rel_y];
% annotation('textarrow',x,y,'String','y = x ')

% figure(h2);
% set(gcf,'Units','centimeters','Position',[5 5 16 9]); %指定plot输出图片的尺寸，xmin，ymin，width，height
% y=y_list(:,1)';
% F_max_val =F_max_list(:,6)';
% plot(y,F_max_val);
% [kmax,id] = findkmax(F_max_val,3);%(寻找前k个最大值点)
% xlabel('末端y坐标');
% ylabel('F函数值');
% title('F随末端y坐标的变化')
% % [val ,pos] = max(F_max_val);
% % text(y(pos),F_max_val(pos),['极值点(',num2str(y(pos)),',',num2str(F_max_val(pos)),')']);
% for i=1:3
%     text(y(id(i)),F_max_val(id(i))-(i-1)*0.006,['极值点',num2str(i),'(',num2str(y(id(i))),',',num2str(F_max_val(id(i))),')'])
%     hold on
%     plot(y(id(i)),F_max_val(id(i)),'-*');
% end
% 
% 
% figure(h3);
% scatter3(F_max_list(:,1),F_max_list(:,3),F_max_list(:,2));
% set(gcf,'Units','centimeters','Position',[5 5 16 9]); %指定plot输出图片的尺寸，xmin，ymin，width，height
% set(gca,'DataAspectRatio',[1,1,1],'PlotBoxAspectRatio',[1,1,1]...,'xLim',[0,1.2],'yLim',[-0.8,0.8],'zLim',[-0.7,0]...
%     ...,'xtick',0.3:0.1:1.2,'ytick',-0.6:0.1:0.6,'ztick',-0.7:0.1:0 ...
%     ...,'xgrid','on','ygrid','on','zgrid','on'...
%     ,'yDir','reverse');
% xlabel('X(m)');
% ylabel('Z(m)');
% zlabel('Y(m)');
% title('每层最大函数值出现的足端坐标点')
% view(-50,30);
% 
% h4 = copy(h3);
% figure(h4);
% view(0,90);


%% P12 求在给定范围内的速度最大值Vmax和Hmin（此处最大范围是考虑实机能否到达，也就是IK的judge为true）
clear all
clc

%整个空间的网格范围是远大于设定范围的，所以目前看来有judge的情况下，应该不会影响我判断Vmax和Hmin
[x_list,z_list] = meshgrid(0.1065:0.005:0.5771,-0.4419:0.005:0.4419);
[m,n]=size(x_list);
%Rb是腿在身体坐标系下的旋转矩阵
 Rb = [1,0,0;
     0,1,0;
     0,0,1];

 Vn=2/3*100*pi; %电机额定转速  单位rad/s

 eb_z=[0,0,1]'; %机身坐标系下指定方向（沿z方向）

h1=figure;
count =1;
y_range=-0.5867:0.01:-0.2199;
y_size = max(size(y_range));
V_max_list=nan(y_size,4); %xyzVmax


for y=y_range
    y_list=ones(m,n)*y;
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
            [q,judge]=IKM([x_list(i,j),y,z_list(i,j)]); %q是向量
            if isreal(q) %q为实数，说明该点为工作空间，进入循环
                if judge ==true
                    J=CalJac(q);
                    Jb=Rb*J;
                    temp_z=(inv(Jb))*eb_z;
                    ve_z=Vn/norm(temp_z,Inf);
                    vel_list_z(i,j) =ve_z;    
                end
            end               
        end
    end
    
    v_max = max(max(vel_list_z));
    if ~isnan(v_max)
        [row, col]=find(vel_list_z==v_max);
        x_coord = x_list(row,col);
        z_coord = z_list(row,col);
        V_max_list(count,4) = v_max;
        V_max_list(count,1) = x_coord;
        V_max_list(count,3) = z_coord;
        V_max_list(count,2) = y;
        
    end
    

    
    figure(h1);
    hold on;
    surf(x_list,z_list,y_list,vel_list_z,'EdgeColor','none');  
    
    count =count+1;
    
    
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
colormap turbo;

Vmax = max(V_max_list(:,4))
ymin = min(V_max_list(:,2))  %这里的数据就是经过筛选的数据了








%% 计算直线交点
clear all
clc
point1 = [-0.5267,0.874135];
point2 = [-0.5167,0.853157];
point3 = [-0.5267,0.849547];
point4 = [-0.5167,0.866277];

[insec_node(1),insec_node(2)] = node(point1,point2,point3,point4);




    



















%% 获取数据，4列分别对应笛卡尔空间的坐标和其速度值
% clear all
% clc
% 
% %预先分配内存
% dq0=30;
% dq1=40;
% dq2=80;
% %遍历范围,注意此处的遍历范围应该是关节空间的范围而不是坐标系，故用q表示
% %注意H_0x的的坐标，B_0y的坐标的区别，区分于实际机器人的参数
% %行程：x推杆0->76mm（坐标系x正方向） y推杆0->-76mm（坐标系y的负方向） R方向正负50°
% k1=16*0.0025/26/2/pi;
% k2=19/50/28;
% 
% %关节空间
% abs_max_q0 = -0.076/k1;
% abs_max_q1 = -0.076/k1;
% abs_max_q2 = 10/k2;
% q0 = linspace(0,abs_max_q0,dq0);
% q1 = linspace(0,abs_max_q1,dq1);
% q2 = linspace(-abs_max_q2,abs_max_q2,dq2);
% 
% data = zeros(dq0*dq1*dq2,4);
% %Rb是腿在身体坐标系下的旋转矩阵
%  Rb = [1,0,0;
%      0,1,0;
%      0,0,1];
% 
%  Vn=2/3*100*pi; %电机额定转速  单位rad/s
% 
%  eb_x=[1,0,0]'; %机身坐标系下指定方向（沿x方向）
% %  eb=[0,1,0]'; %机身坐标系下指定方向（沿y方向）
% %  eb=[0,0,1]'; %机身坐标系下指定方向（沿z方向）
%  
% n=1;
% for i=q0
%     for j=q1
%         for k=q2
%             J=CalJac([i,j,k]);
%             Jb=Rb*J; %机身坐标系下的雅可比矩阵
%             temp=(inv(Jb))*eb_x;    
%             ve=Vn/norm(temp,Inf); 
%             data(n,1:3) = FKM([i,j,k]);
%             data(n,4) = ve;
%             n=n+1
%             
%         end
%     end
% end










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
function [mot_pos3,judge]=IKM(ee_position)
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
DE = 0.39224;
% DE=0.375;
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

%反解中deltaX和deltaY都应该是负的，也就是说这两个变量的有效值为0~-0.076之间，再此加一个判据
if ((-0.076<deltaX) && (deltaX <0) && (-0.065*0.8<deltaY) && (deltaY <0) && (-0.8727< theta0) && (theta0<0.8727))
    judge = true; %judge为真说明该值在行程范围内，可以继续运算
else
    judge = false;
end
    

end


%% 已知两条直线的两个端点，求其交点坐标
% 背景：
% 1、已知：平面上的四个点 X1，Y1，X2，Y2
%         其中，X1与Y1确定一条直线，X2与Y2确定一条直线
% 2、任务：求解X1与Y1确定的直线，X2与Y2确定的直线的交点

function [X, Y]= node( X1,Y1,X2,Y2 )

if X1(1)==Y1(1)
   X=X1(1);
   k2=(Y2(2)-X2(2))/(Y2(1)-X2(1));
   b2=X2(2)-k2*X2(1); 
   Y=k2*X+b2;
end
if X2(1)==Y2(1)
   X=X2(1);
   k1=(Y1(2)-X1(2))/(Y1(1)-X1(1));
   b1=X1(2)-k1*X1(1);
   Y=k1*X+b1;
end
if X1(1)~=Y1(1)&X2(1)~=Y2(1)
   k1=(Y1(2)-X1(2))/(Y1(1)-X1(1));
   k2=(Y2(2)-X2(2))/(Y2(1)-X2(1));
   b1=X1(2)-k1*X1(1);
   b2=X2(2)-k2*X2(1);
    if k1==k2
      X=[];
      Y=[];
   else
   X=(b2-b1)/(k1-k2);
   Y=k1*X+b1;
   end
end
end

%% 寻找数组中前k大的数
function [value, idx]=findkmax(x,k)
value = zeros(1,k);
idx   = zeros(1,k);
m = min(x);
for j = 1:k
    [value(j), idx(j)] = max(x);
    x(idx(j))=m;
end
end



