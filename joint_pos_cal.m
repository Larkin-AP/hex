%计算关节角度
clear all
clc

l1 =[0.335,    0,          0;
    0.403,     0.06094,    0;
    0.403,     0.02136,    0;
    0.383,     -0.032,     0;
    0.51325,   0.09644,    0;
    0.54981,   0,          0;
    0.54992,   -0.08428,   0;
    0.41688,   -0.12609,   0;
    0.38641,   -0.12071,   0;
    0.321,     -0.1225,    0;];

rot = [cos(pi/3), 0,  -sin(pi/3);
       0,         1,  0;
       sin(pi/3), 0,  cos(pi/3)];
l2=l1*rot;
l3=l2*rot;
l4=l3*rot;
l5=l4*rot;
l6=l5*rot;