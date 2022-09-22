%leg2初始位置
clear all 
clc
ground_xyz_ee2=[0.302 -0.393 -0.524 1]';
ground_xyz_ee1=[0.604 -0.393 0 1]';
ground_P_body=eye(4);
leg1_P_body = [1 0 0 -0.3;
    0 1 0 0;
    0 0 1 0;
    0 0 0 1];
leg2_P_body = [0.5 0 -0.866 -0.3;
    0 1 0 0;
    0.866 0 0.5 0;
    0 0 0 1];
body_P_ground = inv(ground_P_body);
real_pm1=leg1_P_body *body_P_ground
real_pm2=leg2_P_body *body_P_ground
leg1_xyz_ee = leg1_P_body *body_P_ground *ground_xyz_ee1
leg2_xyz_ee = leg2_P_body *body_P_ground *ground_xyz_ee2


