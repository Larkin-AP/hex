%讨论v和h的关系

%% 计算ve关于半径r，高度h的关系式
% 以下讨论均对于腿1，即默认z坐标为0
vn=100*pi; %额定速度  rad/s
eb=[1 0 0]'; %x方向单位向量
alpha = 0.5;
beta  = 0.5;

%h0  r0  随便设的参数
h0 = -0.06;
r0 = 0.355;



%半径r和高度h有一个常数换算关系
%%%%%-----------这里x,y均是末端在腿坐标系下的坐标---------------%%%%%
syms x y

x=157.3536;
y=-340.5121;

vector_q = Inverse_kinematic([x,y,0]);
q0=vector_q(1);
q1=vector_q(2);
q2=vector_q(3);

Jl=CalJac(vector_q);

%Rb是腿在身体坐标系下的旋转矩阵
 Rb = eye(3);
 Jb=Rb*Jl;
 temp=eye(3)/Jb*eb;
 ve=vn/norm(temp,Inf);
 
 h=h0-y;
 
 
 %目标函数
 F=alpha *ve +beta*h 
