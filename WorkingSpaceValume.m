%% 计算工作空间体积
%思路 参考潘老师论文思路，指定步长ds，产生值，代入反解中，如果解正确，记录该点，最后计算n*ds^3
clear all
clc


%传入的点是在腿坐标系下的坐标，此处单位为mm
%行程：x推杆0->76mm y推杆0->76mm R方向正负50°
%转换到q对应为：
%dx=0，dy=0     ee=(165.5,-369.67)
%dx=0, dy=-76   ee=(237.54,-586.68)
%dx=76,dy=0     ee=(496.12,-215.92)
%dx=76,dy=-76   ee=(580.72,-329.75)
%所以外切立方体即为各个方向的最值，加上旋转，实际为类似于扇形的体积
%165.5<x<580.72  -586.68<y<-215.92 -444.86<z<444.86
ds=0.01; %ds为步长，设为0.01m
% num=((580.72-165.5)/ds * (-215.92+586.68)/ds * (444.48+444.86)/ds);

coor_Scope = [0.1065,0.5771;
    -0.5867,-0.2199;
    -0.4419,0.4419];


c=zeros(142045000,3);
n1=1;
n2=1;
%三重循环
for i=coor_Scope(1,1):ds:coor_Scope(1,2)
    for j=coor_Scope(2,1):ds:coor_Scope(2,2)
        for k=coor_Scope(3,1):ds:coor_Scope(3,2)
            [a,judge]=Inverse_kinematic([i,j,k]);
            if ( imag(a)==0)
                if (judge == true)
                    c(n1,:)=[i,j,k];
                    n1=n1+1
                end

            end
           
        end
    end
end
% a(all(a==0,2),:) = [];  %去掉里面的全0行，避免无效筛选
c(all(c==0,2),:) = []; 
            


% %% a为要验证的数组，传入数组a，运行后得到的a即为剔除复数剩下的数组
% n2=0;
% b=zeros(n1,3);
% 
% %legnth返回数组的最大维度
% for i=1:length(a)
%     if ( imag(a(i,:))==0)  %imag()返回数组中每个元素的虚部
%             n2=n2+1
%             b(n2,:)=a(i,:); %b记录实数
%     end
% end
% b(all(b==0,2),:) = [];  %去掉里面的全0行，避免无效筛选
% if (n2~=0)
%     a=b;    %把实数返还给a
% else
%     disp('No real number in array a');
% end

%% 
%旧
%当ds=10  V = 0.1097m^3  ita=0.8045
%当ds=5   V = 0.1081m^3
%当ds=2   V = 0.1070m^3  ita=0.7844
%当ds=1   V = 0.1066m^3  ita=0.7816

%新
%当ds=0.01  V=0.1243
%计算体积
V = (n1-1)*ds^3
%计算实际空间可达率
ita = V/((coor_Scope(1,2)-coor_Scope(1,1))*(coor_Scope(2,2)-coor_Scope(2,1))*(coor_Scope(3,2)-coor_Scope(3,1)))



%% 画指定零方体

A=[coor_Scope(1,1),coor_Scope(2,2),coor_Scope(3,2);
coor_Scope(1,1),coor_Scope(2,2),coor_Scope(3,1);
coor_Scope(1,2),coor_Scope(2,2),coor_Scope(3,1);
coor_Scope(1,2),coor_Scope(2,2),coor_Scope(3,2);
coor_Scope(1,1),coor_Scope(2,1),coor_Scope(3,2);
coor_Scope(1,1),coor_Scope(2,1),coor_Scope(3,1);
coor_Scope(1,2),coor_Scope(2,1),coor_Scope(3,1);
coor_Scope(1,2),coor_Scope(2,1),coor_Scope(3,2)];
d=[1 2 3 4 8 5 6 7 3 2 6 5 1 4 8 7];
plot3(A(d,3),A(d,1),A(d,2));
xlabel('z');
ylabel('x');
zlabel('y');
hold on
set(gca,'XDir','reverse');        %将x轴方向设置为反向(从右到左递增)。
set(gca,'YDir','reverse');        %将x轴方向设置为反向(从右到左递增)。
scatter3(c(:,3),c(:,1),c(:,2),3,'.','b');
% view(3);
view([-135,-45]);
rotate3d;

