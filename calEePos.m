%计算关节旋转位置
clear all
clc


digits (5);
 leg1_ee=[
                    0.0469486,  -0.0035000,  0.0271058; 
                    0.0469486,  -0.0035000,  0.0271058; 
                    0.0360601,  -0.0209923,  0.0348780; 
                    0.0326315,  -0.0295093,  0.0527093; 
                    0.0362146,  -0.0381481,  0.0349672; 
                    0.0483898,  -0.0381481,  0.0138791; 
                    0.0637436,  -0.0294807,  0.0043196;
                    0.0482353,  -0.0209923,  0.0137899 
                    
 ];


ry60=[cos(-pi/3),0,sin(-pi/3);
0 1 0;
-sin(-pi/3) 0 cos(-pi/3)];

m = max(size(leg1_ee));
leg2_ee=zeros(m,3);
leg3_ee=zeros(m,3);
leg4_ee=zeros(m,3);
leg5_ee=zeros(m,3);
leg6_ee=zeros(m,3);



for i=1:m
    temp=ry60*leg1_ee(i,:)';
    leg2_ee(i,:)=temp';
end

for i=1:m
    temp=ry60*leg2_ee(i,:)';
    leg3_ee(i,:)=temp';
end

for i=1:m
    temp=ry60*leg3_ee(i,:)';
    leg4_ee(i,:)=temp';
end

for i=1:m
    temp=ry60*leg4_ee(i,:)';
    leg5_ee(i,:)=temp';
end

for i=1:m
    temp=ry60*leg5_ee(i,:)';
    leg6_ee(i,:)=temp';
end

ee=zeros(6,3);
a=[0.0469684,         -0.0530806,         0.0271058];
for i=1:5
    ee(1,:) = a;
    ee(i+1,:) = ry60*ee(i,:)';
end


j1_o=[1.2688,4.4327,5.2670];%测量值
j1=j1_o/norm(j1_o); %单位化
j1

j2_o = [4.5289,3.6151,3.9269];
j2 = j2_o/norm(j2_o);




l1_joint = [0.8492,    -0.1960,    0.4903;
    0.5000,         0,    -0.8660;
    0.7422,    -0.6332,    -0.2192;
    0.9937,    0.0095,    0.1115;
    0.7307,    0.6442,    -0.2259;
    0.1698,    0.6442,    0.7458;
    0.5934,    0.0095,    0.8048;
    0.1813,    -0.6332,    0.7524];

l2_joint=zeros(m,3);
l3_joint=zeros(m,3);
l4_joint=zeros(m,3);
l5_joint=zeros(m,3);
l6_joint=zeros(m,3);

for i=1:m
    temp=ry60*l1_joint(i,:)';
    l2_joint(i,:)=temp';
end

for i=1:m
    temp=ry60*l2_joint(i,:)';
    l3_joint(i,:)=temp';
end

for i=1:m
    temp=ry60*l3_joint(i,:)';
    l4_joint(i,:)=temp';
end

for i=1:m
    temp=ry60*l4_joint(i,:)';
    l5_joint(i,:)=temp';
end

for i=1:m
    temp=ry60*l5_joint(i,:)';
    l6_joint(i,:)=temp';
end



