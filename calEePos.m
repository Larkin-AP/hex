%计算关节旋转位置
 leg1_ee=[0.3 0 0;
0.368 0.088 0;
0.368 0.012 0;
0.348 -0.032 0;
0.49931 0.07045 0;
0.52379 -0.02675 0;
0.41987,   -0.09621,   0;
0.3676,    -0.12872,   0;
0.3427,    -0.12133,   0;
0.286,     -0.10345,   0];

ry60=[cos(pi/3),0,sin(pi/3);
0 1 0;
-sin(pi/3) 0 cos(pi/3)];

leg2_ee=zeros(10,3);
leg3_ee=zeros(10,3);
leg4_ee=zeros(10,3);
leg5_ee=zeros(10,3);
leg6_ee=zeros(10,3);

for i=1:10
    temp=ry60*leg1_ee(i,:)';
    leg2_ee(i,:)=temp';
end

for i=1:10
    temp=ry60*leg2_ee(i,:)';
    leg3_ee(i,:)=temp';
end

for i=1:10
    temp=ry60*leg3_ee(i,:)';
    leg4_ee(i,:)=temp';
end

for i=1:10
    temp=ry60*leg4_ee(i,:)';
    leg5_ee(i,:)=temp';
end

for i=1:10
    temp=ry60*leg5_ee(i,:)';
    leg6_ee(i,:)=temp';
end


