%画图尝试
%% try 1
clear all
clc

xAxis = linspace(-1,2.5,1000);
yAxis = linspace(-1,2.5,1000);
zAxis = linspace(-1,2.5,1000);
[ X,Y ] = meshgrid(xAxis,yAxis); %meshgrid相当于for循环遍历，但感觉只适用于可以表达出坐标式的函数
Z = X.^2 + 5*Y.^2 ;
contour(X,Y,Z,30) %等高线图

%% try 2
clear al
clc

x = [1 2];
y = [2 3];
z = [3 4];
[X, Y, Z] = meshgrid(x,y,z);
ret = f1([X,Y,Z]);

%% try 3
clear all
clc

xAxis = linspace(-1,2.5,100);
yAxis = linspace(-1,2.5,100);
zAxis = linspace(-1,2.5,100);
[X, Y, Z] = meshgrid(xAxis,yAxis,zAxis);
f = X.^4 - 2*X.^2 + Y.^2 + 2*Y.*Z +2*Z.^2;
x_planes = [1 2];
y_places = 0;
z_planes = [0.5 1];
slice(X, Y, Z, f, x_planes, y_places, z_planes);

%% try 4 貌似这种方法可行，用slice加上色棒的方法
%目前的问题是，关于遍历空间，如果没法表示出f的具体表达式，只能用f=f(x,y,z)的形式，能不能用meshgrid
clear all
clc

x=linspace(-1,1,11);
y=linspace(-2,2,21);
z=linspace(-3,3,31);

[X,Y,Z] = meshgrid(x,y,z);
V = X.*Y.*Z;

xslice = [-0.5,0.5];   
yslice = [];
zslice = 0;
slice(X,Y,Z,V,xslice,yslice,zslice)
xlabel('x');
ylabel('y');
zlabel('z');
colorbar

%% try 5
clear all
clc

x=[1,2 ];
y=[2 3,5];
z=[4 5 6,7];
[X,Y,Z]=meshgrid(x,y,z);
v=X.*Y.*Z;
xslice = [-1.2,0.8,2];   
yslice = [];
zslice = 0;
slice(X,Y,Z,v,xslice,yslice,zslice)
%% try 6
clear all
clc

[X,Y] = meshgrid(1:0.5:10,1:20);
Z = sin(X) + cos(Y);
C = X.*Y;
surf(X,Y,Z,C)
colorbar

%% try 7
clear all
clc

figure
peaks(25);

%% try 8
clear all
clc

[X,Y,Z] = peaks(25);
mesh(X,Y,Z)

%% try 9
N = 10 ;
x = linspace(0,1,N) ;
y = linspace(0,1,N) ;
z = linspace(0,1,N) ;
[X,Y,Z] = meshgrid(x,y,z) ;
f = X.*Y.*Z ;
figure(1)
hold on
for i = 1:N
    surf(X(:,:,i),Y(:,:,i),Z(:,:,i),f(:,:,i))
end

%% try 9
clear all
clc

x = 1:100;
y = 1:100;
z = 1:100;
f = (x.^2).*y./z.^(3/2);
surface([x;x],[y;y],[z;z],[f;f],'facecol','no','edgecol','interp','linew',2);
view(3);

%% try 10 等高线图
clear all
clc
x = linspace(-2*pi,2*pi,40);
y = linspace(0,4*pi,50);
[X,Y] = meshgrid(x,y);
Z = sin(X) + cos(Y);
contourf(X,Y,Z,10)
colorbar

%% try 11
clear all
clc

f = @(x,y,z) x.*y.*z.*log(1+x.^2+y.^2+z.^2)-10;% 函数表达式
[x,y,z] = meshgrid(-10:.2:10,-10:.2:10,-10:.2:10);% 画图范围
v = f(x,y,z);
h = patch(isosurface(x,y,z,v,0)); 
isonormals(x,y,z,v,h)
set(h,'FaceColor','r','EdgeColor','none');
xlabel('x');ylabel('y');zlabel('z'); 
alpha(1)
grid on; view([1,1,1]); axis equal; camlight; lighting gouraud

%% try 12
clear all
clc
[x_list,z_list]=meshgrid(0.3:0.01:1.2,-0.8:0.01:0.8);
[m,n]=size(x_list);
y=0.2;
y_list=ones(m,n)*y;
f_list = x_list.*z_list.^2;
f_list = nan(m,n);
surf(x_list,z_list,y_list,f_list,'EdgeColor','none');    
%% try 13
a = i
if ~isreal(a)
    a=a*2;
end







%% function 1
%随便写一个函数
function ret = f1(input)
x = input(1);
y = input(2);
z = input(3);
ret = (x+1)^2+y+z^3;

end