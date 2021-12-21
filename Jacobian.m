%计算雅可比矩阵
%X = J*theta
%X是末端位置，theta是电机位置
%目前theta输入是[l1.l2.alpha],输出是末端位置X=[x,y,z]
syms l1 Hx H0
l1=Hx-H0