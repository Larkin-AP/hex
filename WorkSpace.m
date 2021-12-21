% l1=500;l2=600;l3=400;l4=191.03;
% t1=linspace(-180,180,90)*pi/180;
% t2=linspace(-90,90,90)*pi/180;
% d3=linspace(-200,200,90);
% t4=linspace(-180,180,90)*pi/180;
% [T1,T2,D3]=ndgrid(t1,t2,d3);  % This will create matrices of 90x90x90 for each variable
% xM = round((-cos(T1).*cos(T2)).*((D3 + l2 + l3 + l4)));
% yM = round((-cos(T2).*sin(T1)).*(D3 + l2 + l3 + l4));
% zM = round((l1 - l4.*sin(T2) - sin(T2).*(D3 + l2 + l3)));
% % plot3(xM(:),yM(:),zM(:),'.') % This is the plot type you should be using.
% % With a '.' as an argument to show only locations and not lines
% % Also, (:) converts any matrix into a list of its elements in one single column.


%draw workspace of hexapod
clear all
clc
tic
x = linspace(-10,10,10);
y = linspace(-10,10,10);
r = linspace(-10,10,10);
nx = length(x);
ny = length(y);
nr = length(r);
count = 0;
Mmax=nx*ny*nr;
ee = zeros(Mmax,3);
M = 1;
% [X,Y,R] = ndgrid(x,y,r);
% xE = Forward_kinematics([X,Y,R]);
% yE = Forward_kinematics([X,Y,R]);
% rE = Forward_kinematics([X,Y,R]);
for i=1:nx
    for j=1:ny
        for k=1:nr
            ee(M,:) = Forward_kinematics([i,j,k]);
            plot3(ee(M,1),ee(M,2),ee(M,3),'.')
            hold on
            M =M+1;
            count =count +1
        end
    end
end
toc
% disp('Run time :' ,num2str(toc));
% plot3(ee(1),ee(2),ee(3),'.')
            
% plot3(xE(:),yE(:),rE(:),'.') % This is the plot type you should be usin





