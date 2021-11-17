#include"kinematics.h"
#include<iostream>
#include<aris.hpp>

double PI = aris::PI;

//身体在腿坐标系下的变换矩阵，腿坐标系建立在转轴与底板地面的交点上，而非反解中的A点，此处需要注意
//身体坐标系在1腿的下的变换矩阵，腿的坐标系均与地面坐标系朝向一致  即Leg_P_Body


double PL1[16] =
{
	 1, 0,  0,  -FRONTX,
	 0, 1,  0,  0,
	 0, 0,  1,	0,
	 0, 0,  0,  1
};

double PL2[16] =
{
     0.5,		0,		-0.866025,		-FRONTX,
     0,			1,		0,              0,
     0.866,		0,		0.5,            0,
     0,			0,		0,              1
};

double PL3[16] =
{
     -0.5,		0,		-0.866025,		-FRONTX,
	 0,			1,		0,			0,
     0.866025,		0,		-0.5,		0,
	 0,			0,		0,			1
};

double PL4[16] =
{
	 -1,		0,		0,			-FRONTX,
	 0,			1,		0,			0,
	 0,			0,		-1,			0,
	 0,			0,		0,			1
};

double PL5[16] =
{
     -0.5,		0,		0.866025,		-FRONTX,
	 0,			1,		0,			0,
     -0.866025,	0,		-0.5,		0,
	 0,			0,		0,			1
};

double PL6[16] =
{
     0.5,		0,		0.866025,		-FRONTX,
	 0,			1,		0,			0,
     -0.866025,	0,		0.5,		0,
	 0,			0,		0,			1
};

 //运动学反解
 //ee_position为末端坐标，按顺序为xyz
 //mot_pos即为驱动杆件的坐标变化值，对应坐标为xyR
 //leg_inverse_kinematics函数输入指定末端所到位置ee_position，把求解的结果放在mot_pos
 //ee_position要求是在"""腿坐标系下的坐标"""，故在传进来之前要求进行坐标变换，传入坐标依次为xyz
 //腿坐标系与身体坐标系同向
 auto legInverseKinematics(double *ee_position, double *mot_pos)->void
 {
	 //此处坐标原点在 转轴与底板地面的交点
	 //mot_pos[2] = -atan2(ee_position[2], ee_position[0]); //竖直转动轴 ###此处暂时不加一个负号
	 double theta0 = -atan2(ee_position[2], ee_position[0]);  //竖直转动轴转动的角度，还需要转换到电机轴上
	 mot_pos[2] = 50 * 28 / 19 * theta0;  //减速箱减速比50：1，带传动比 28：19
	 double x0 = sqrt(ee_position[2] * ee_position[2] + ee_position[0] * ee_position[0]); //反解所在平面的x值
	 double y0 = ee_position[1]; //反解所在平面的y值

	 double x = x0 - PA_X;
	 double y = y0 + PA_Y;
	 

	 double AN = sqrt(x * x + y * y);
	 double angle_NCA = (acos((NC * NC + AC * AC - AN * AN) / 2 / NC / AC));
	 double angle_CAN = (acos((AC * AC + AN * AN - NC * NC) / 2 / AC / AN));
	 double angle_CAG = PI - angle_NCA;
	 double angle_NAG = angle_CAG - angle_CAN;
	 double angle_NAJ = (atan(-x / y));
	 double angle_GAJ = angle_NAJ - angle_NAG;
	 double Gx = AG * sin(angle_GAJ);
	 double Gy = -AG * cos(angle_GAJ);
	 double vector_AG1 = Gx;
	 double vector_AG2 = Gy;
	 double Hy = -AJ;
	 double Hx = 0;
	 if (Gy < Hy)
	 {
		 double HK = Hy - Gy;
		 double angle_HGK = asin(HK / GH);
		 double GK = GH * cos(angle_HGK);
		 Hx = Gx - GK;
	 }
	 else if (Gy > Hy)
	 {
		 double GK = Gy - Hy;
		 double angle_GHK = asin(GK / GH);
		 double HK = GH * cos(angle_GHK);
		 Hx = Gx - HK;
	 }
	 else
	 {
		 Hx = Gx - GH; 
	 }
	 //mot_pos[0] = Hx - H_0x; //X方向推杆
	 double deltaX = Hx - H_0x; //x方向推杆变化值，还需要转换到电机的旋转变换值
     //std::cout << "Hx = " << Hx << std::endl;
	 mot_pos[0] = 26 / 16 * deltaX / 0.0025; //导程为2.5mm，转换到m，带传动传动比为26:16  电机输出量

	 

	 double vector_AN1 = x;
	 double vector_AN2 = y;
	 double vector_CN1 = NC / AG * vector_AG1;
	 double vector_CN2 = NC / AG * vector_AG2;
	 double vector_AC1 = vector_AN1 - vector_CN1;
	 double vector_AC2 = vector_AN2 - vector_CN2;
	 double vector_GF1 = GF / AC * vector_AC1;
	 double vector_GF2 = GF / AC * vector_AC2;
	 double vector_AF1 = vector_AG1 + vector_GF1;
	 double vector_AF2 = vector_AG2 + vector_GF2;
	 double FL = vector_AF1 - LM;
	 double BL = sqrt(BF * BF - FL * FL);
	 double By = vector_AF2 + BL;
	 //mot_pos[1] = -(By - B_0y); //Y方向推杆  向下推动？ 我也不懂？
	 double deltaY = -(By - B_0y); //Y方向推杆变化值，还需转换到电机上  向下推动？ 我也不懂？
     //std::cout << "By = " << By <<std::endl;
	 mot_pos[1] = 26 / 16 * deltaY / 0.0025; //导程为2.5mm，转换到m，带传动传动比为26:16   电机输出量

 }

 //在坐标变换后求解反解
 //input是输入的末端坐标（世界坐标系下）
 //Body_P_Ground是地面在身体坐标系的表达，此处初始位置重合，故为4*4单位阵
 //Leg_P_Body是身体坐标系在腿坐标系的表达
 //最后需要的腿末端在腿坐标系下的表达xyz_in_leg
 //Leg_xyz_ee =Leg_P_Body * Body_P_Ground * Ground_xyz_ee(input4*1)
 //A在B中表示的变换矩阵与B在A中表示的变化矩阵互为逆，即Body_P_Ground * Ground_P_Body = I
 //inverseLeg函数是传入（末端在地面的xyz坐标，身体在地面的坐标，结果保存的地方（电机推杆变化值））



 auto inverseLeg(double *ground_xyz_ee, double *ground_p_body, double *input)->int
 {
	 double real_pm_l1[16] = { 0 };
	 double real_pm_l2[16] = { 0 };
	 double real_pm_l3[16] = { 0 };
	 double real_pm_l4[16] = { 0 };
	 double real_pm_l5[16] = { 0 };
	 double real_pm_l6[16] = { 0 };
	 aris::dynamic::s_pm_dot_inv_pm(PL1, ground_p_body, real_pm_l1);  //Leg_P_Ground
	 aris::dynamic::s_pm_dot_inv_pm(PL2, ground_p_body, real_pm_l2);  //Leg_P_Ground
	 aris::dynamic::s_pm_dot_inv_pm(PL3, ground_p_body, real_pm_l3);  //Leg_P_Ground
	 aris::dynamic::s_pm_dot_inv_pm(PL4, ground_p_body, real_pm_l4);  //Leg_P_Ground
	 aris::dynamic::s_pm_dot_inv_pm(PL5, ground_p_body, real_pm_l5);  //Leg_P_Ground
	 aris::dynamic::s_pm_dot_inv_pm(PL6, ground_p_body, real_pm_l6);  //Leg_P_Ground

	 double xyz_in_leg_l1[18] = { 0 }; //每条腿末端在各自腿坐标系下的表达，写在一个矩阵中
	 //该函数是输入（位姿变化矩阵，一个坐标，输出转换后的坐标3*1）
	 aris::dynamic::s_pp2pp(real_pm_l1, ground_xyz_ee + 0 * 3, xyz_in_leg_l1 + 0 * 3); //l1
	 aris::dynamic::s_pp2pp(real_pm_l2, ground_xyz_ee + 1 * 3, xyz_in_leg_l1 + 1 * 3); //l2
	 aris::dynamic::s_pp2pp(real_pm_l3, ground_xyz_ee + 2 * 3, xyz_in_leg_l1 + 2 * 3); //l2
	 aris::dynamic::s_pp2pp(real_pm_l4, ground_xyz_ee + 3 * 3, xyz_in_leg_l1 + 3 * 3); //l2
	 aris::dynamic::s_pp2pp(real_pm_l5, ground_xyz_ee + 4 * 3, xyz_in_leg_l1 + 4 * 3); //l2
	 aris::dynamic::s_pp2pp(real_pm_l6, ground_xyz_ee + 5 * 3, xyz_in_leg_l1 + 5 * 3); //l2


	 //得到mot_pos
	 legInverseKinematics(xyz_in_leg_l1 + 0 * 3, input + 0 * 3);
	 legInverseKinematics(xyz_in_leg_l1 + 1 * 3, input + 1 * 3);
	 legInverseKinematics(xyz_in_leg_l1 + 2 * 3, input + 2 * 3);
	 legInverseKinematics(xyz_in_leg_l1 + 3 * 3, input + 3 * 3);
	 legInverseKinematics(xyz_in_leg_l1 + 4 * 3, input + 4 * 3);
	 legInverseKinematics(xyz_in_leg_l1 + 5 * 3, input + 5 * 3);
//     for (int i = 0; i<18;++i){
//         std::cout << input[i] << "\t";
//         if (i%3==2){
//             std::cout << std::endl;
//         }
//     }
	 
	 return 0;
 }


