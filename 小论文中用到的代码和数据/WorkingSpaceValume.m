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

%这个范围是怎么产生的
coor_Scope = [0.2065,0.5765;  %x坐标
    -0.5867,-0.2267;  %y坐标
    -0.4419,0.4381];  %z坐标范围


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

min(c(:,1))
min(c(:,2))
min(c(:,3)) 

max(c(:,1))
max(c(:,2))
max(c(:,3)) 


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

h1 = figure;
% h2 = figure;

A=[coor_Scope(1,1),coor_Scope(2,2),coor_Scope(3,2);
coor_Scope(1,1),coor_Scope(2,2),coor_Scope(3,1);
coor_Scope(1,2),coor_Scope(2,2),coor_Scope(3,1);
coor_Scope(1,2),coor_Scope(2,2),coor_Scope(3,2);
coor_Scope(1,1),coor_Scope(2,1),coor_Scope(3,2);
coor_Scope(1,1),coor_Scope(2,1),coor_Scope(3,1);
coor_Scope(1,2),coor_Scope(2,1),coor_Scope(3,1);
coor_Scope(1,2),coor_Scope(2,1),coor_Scope(3,2)];
d=[1 2 3 4 8 5 6 7 3 2 6 5 1 4 8 7];

figure(h1);
h1 = plot3(A(d,3),A(d,1),A(d,2));
hold on
set(gca,'FontSize',24,'FontName','Times New Roman')

xlabel('z(m)','FontSize',32);
ylabel('x(m)','FontSize',32);
zlabel('y(m)','FontSize',32);



face1 = [coor_Scope(3,1) coor_Scope(3,1) coor_Scope(3,1) coor_Scope(3,1); %z
    coor_Scope(1,1) coor_Scope(1,2) coor_Scope(1,2) coor_Scope(1,1);  %x
    coor_Scope(2,1) coor_Scope(2,1) coor_Scope(2,2) coor_Scope(2,2)]; %y

face2 = [coor_Scope(3,1) coor_Scope(3,2) coor_Scope(3,2) coor_Scope(3,1); %z
    coor_Scope(1,1) coor_Scope(1,1) coor_Scope(1,2) coor_Scope(1,2);  %x
    coor_Scope(2,2) coor_Scope(2,2) coor_Scope(2,2) coor_Scope(2,2)]; %y

face3 = [coor_Scope(3,2) coor_Scope(3,2) coor_Scope(3,2) coor_Scope(3,2); %z
    coor_Scope(1,1) coor_Scope(1,2) coor_Scope(1,2) coor_Scope(1,1);  %x
    coor_Scope(2,1) coor_Scope(2,1) coor_Scope(2,2) coor_Scope(2,2)]; %y

face4 = [coor_Scope(3,1) coor_Scope(3,2) coor_Scope(3,2) coor_Scope(3,1); %z
    coor_Scope(1,1) coor_Scope(1,1) coor_Scope(1,2) coor_Scope(1,2);  %x
    coor_Scope(2,1) coor_Scope(2,1) coor_Scope(2,1) coor_Scope(2,1)]; %y

face5 = [coor_Scope(3,1) coor_Scope(3,2) coor_Scope(3,2) coor_Scope(3,1); %z
    coor_Scope(1,1) coor_Scope(1,1) coor_Scope(1,1) coor_Scope(1,1);  %x
    coor_Scope(2,1) coor_Scope(2,1) coor_Scope(2,2) coor_Scope(2,2)]; %y

face6 = [coor_Scope(3,1) coor_Scope(3,2) coor_Scope(3,2) coor_Scope(3,1); %z
    coor_Scope(1,2) coor_Scope(1,2) coor_Scope(1,2) coor_Scope(1,2);  %x
    coor_Scope(2,1) coor_Scope(2,1) coor_Scope(2,2) coor_Scope(2,2)]; %y


patch([face1(1,:)], [face1(2,:)], [face1(3,:)], 'b', 'FaceAlpha',0.1) 
patch([face2(1,:)], [face2(2,:)], [face2(3,:)], 'b', 'FaceAlpha',0.1) 
patch([face3(1,:)], [face3(2,:)], [face3(3,:)], 'b', 'FaceAlpha',0.1) 
patch([face4(1,:)], [face4(2,:)], [face4(3,:)], 'b', 'FaceAlpha',0.1) 
patch([face5(1,:)], [face5(2,:)], [face5(3,:)], 'b', 'FaceAlpha',0.1) 
patch([face6(1,:)], [face6(2,:)], [face6(3,:)], 'b', 'FaceAlpha',0.1) 

% patch([X(1:6) flip(X(1:6))], [Y(1:6) flip(Y(1:6))], [Z(1:6) flip(Z(1:6))], 'b', 'FaceAlpha',0.1) %flip 为翻转目标中的元素  facealpha为0-1之间的透明度设置。
% kp = 2;
% patch([X((1:6)+kp) flip(X((1:6)+kp))], [Y((1:6)+kp) flip(Y((1:6)+kp))], [Z((1:6)+kp) flip(Z((1:6)+kp))], 'b', 'FaceAlpha',0.1) %X(1:4)为1-4，X((1:4)+2) 为(3:6)
% kp = 10;
% patch([X((1:6)+kp) flip(X((1:6)+kp))], [Y((1:6)+kp) flip(Y((1:6)+kp))], [Z((1:6)+kp) flip(Z((1:6)+kp))], 'b', 'FaceAlpha',0.1)



set(gca,'XDir','reverse');        %将x轴方向设置为反向(从右到左递增)。
set(gca,'YDir','reverse');        %将x轴方向设置为反向(从右到左递增)。
set(gca,'ytick',[0.3 0.5]);
set(gca,'ztick',[-0.5 -0.3]);


% set(gca,'ytick',[]);
% set(gca,'ztick',[]);
% set(gca,'xtick',[]);

% set(gca,'xticklabel','');
% set(gca,'yticklabel','');
% set(gca,'zticklabel','');

set(gcf,'Units','centimeters','Position',[5 5 24 18]);
% scatter3(c(:,3),c(:,1),c(:,2),3,'.','b');
% view(3);
shp = alphaShape(c(:,3),c(:,1),c(:,2));
shp.Alpha =0.2
h1 = plot(shp,'FaceColor',	'#D95319'); 


% colormap hot
axis equal
% axis square
grid on
view([-135,30]);
% view([-90,0]);
% view([0,-90]);
rotate3d;

%%
%尝试在三维平明使用patch函数
X_try = [1 3 3 1]';
Y_try = [1 2 3 4]';
Z_try = -[1 2 2 1]';

figure;
patch(X_try,Y_try,Z_try, 'r', 'FaceAlpha',0.25)
view([135,30]);








%% 
% shp = alphaShape(c(:,1),c(:,2),c(:,3),10);
% c_size = max(size(c));
% col = linspace(0,1,c_size)';
% 
% shp.Alpha =0.1;
% pl=plot(shp)
% pl.Facecolor = 'r'
% 
% 
% % plot(shp,'FaceColor',yellowgreen)
% % plot(shp)
% % colormap('winter')
% % plot(shp,'color',[col col col]);
% 
% 
% % h.FaceColor = 'r'
% axis equal
% 
% % k=128
% colormap(summer(k));

% a= linspace(1,256,256);
% b= linspace(1,256,256);
% plot(a,b,'Color',[0,1,1])




