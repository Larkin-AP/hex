%draw workspace of hexapod
clear all
clc
tic

%此处xyr对应的是电机值
%数据测量是不包括足垫厚度的
%x丝杠距离传感器的距离是2->78
%y丝杠距离传感器的距离是2->-76
%r行程是正负50°转换为弧度 正负0.872
%转换到电机值
%deltaX为0->m0=-delta*26*2*pi/16/2.5=76*26*2*pi/16/2.5=-310.3890
%deltaY为0->m1=-delta*26*2*pi/16/2.5=76*26*2*pi/16/2.5=310.3890
%r为-theta->theta ,theta=m*50*28/19=0.872*50*28/19=64.2526
x = linspace(0,-310.3890,20);
y = linspace(0,-310.3890,20);
r = linspace(-64.2526,64.2526,50);
nx = length(x);
ny = length(y);
nr = length(r);

Mmax=nx*ny*nr;
ee = zeros(Mmax,3);
M = 1;

% [X,Y,R] = ndgrid(x,y,r);
% xE = Forward_kinematics([X,Y,R]);
% yE = Forward_kinematics([X,Y,R]);
% rE = Forward_kinematics([X,Y,R]);
%此处坐标做点处理，因为不太会设置三维坐标系位置
%竖直为y（上为正）水平为x（右为正）纵向为z（外为正）
figure(1)
for i=x
    for j=y
        for k=r
            ee(M,:) = Forward_kinematics([i,j,k]);
            M =M+1
        end
    end
end

figure(1)
scatter3(ee(:,3),ee(:,1),ee(:,2),3,'.','b');
axis equal
axis tight
grid on;
xlim([-0.400,0.400]);%Z
ylim([0,0.600]);%X
zlim([-0.600,0]);%Y
set(gca,'XDir','reverse'); 
set(gca,'YDir','reverse'); 
xlabel('Z(mm)');
ylabel('X(mm)');
zlabel('Y(mm)');
view([135,45]);


figure(2)
scatter3(ee(:,3),ee(:,1),ee(:,2),3,'.','b');
axis equal
axis tight
grid on;
xlim([-0.400,0.400]);%Z
ylim([0,0.600]);%X
zlim([-0.600,0]);%Y
xlabel('Z(mm)');
ylabel('X(mm)');
zlabel('Y(mm)');
view([0,-90]);


figure(3)
scatter3(ee(:,3),ee(:,1),ee(:,2),3,'.','b');
axis equal
axis tight
grid on;
xlim([-0.400,0.400]);%Z
ylim([0,0.600]);%X
zlim([-0.600,0]);%Y
xlabel('Z(mm)');
ylabel('X(mm)');
zlabel('Y(mm)');
view([0,90]);

figure(4)
scatter3(ee(:,3),ee(:,1),ee(:,2),3,'.','b');
axis equal
axis tight
grid on;
xlim([-0.400,0.400]);%Z
ylim([0,0.600]);%X
zlim([-0.600,0]);%Y
set(gca,'XDir','reverse'); 
set(gca,'YDir','reverse'); 
xlabel('Z(mm)');
ylabel('X(mm)');
zlabel('Y(mm)');
view([-135,-45]);

toc

%确定工作空间最大值
xMax = max(ee(:,1));
xMin = min(ee(:,1));
yMax = max(ee(:,2));
yMin = min(ee(:,2));
zMax = max(ee(:,3));
zMin = min(ee(:,3));



%由此计算的工作空间各坐标分布情况如下：
coor_Scope = [xMin,xMax;
    yMin,yMax;
    zMin,zMax]

%{
    间隔为50时，
    coor_Scope =

    0.1065    0.5771
   -0.5867   -0.2199
   -0.4419    0.4419
    
    
    %}





