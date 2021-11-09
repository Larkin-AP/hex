#include"kinematics.h"
#include<iostream>
#include<aris.hpp>

double PI = aris::PI;

//������������ϵ�µı任����������ϵ������ת����װ����Ľ����ϣ����Ƿ����е�A�㣬�˴���Ҫע��
//��������ϵ��1�ȵ��µı任�����ȵ�����ϵ�����������ϵ����һ��  ��Leg_P_Body


double PL1[16] =
{
	 1, 0,  0,  -FRONTX,
	 0, 1,  0,  0,
	 0, 0,  1,	0,
	 0, 0,  0,  1
};

double PL2[16] =
{
	 0.5,		0,		-0.866,		-FRONTX,
	 0,			1,		0,			0,
	 0.866,		0,		0.5,		0,
	 0,			0,		0,			1
};

double PL3[16] =
{
	 -0.5,		0,		-0.866,		-FRONTX,
	 0,			1,		0,			0,
	 0.866,		0,		-0.5,		0,
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
	 -0.5,		0,		0.866,		-FRONTX,
	 0,			1,		0,			0,
	 -0.866,	0,		-0.5,		0,
	 0,			0,		0,			1
};

double PL6[16] =
{
	 0.5,		0,		0.866,		-FRONTX,
	 0,			1,		0,			0,
	 -0.866,	0,		0.5,		0,
	 0,			0,		0,			1
};

 //�˶�ѧ����
 //ee_positionΪĩ�����꣬��˳��Ϊxyz
 //mot_pos��Ϊ�����˼�������仯ֵ����Ӧ����ΪxyR
 //leg_inverse_kinematics��������ָ��ĩ������λ��ee_position�������Ľ������mot_pos
 //ee_positionҪ������"""������ϵ�µ�����"""�����ڴ�����֮ǰҪ���������任��������������Ϊxyz
 //������ϵ����������ϵͬ��
 auto legInverseKinematics(double *ee_position, double *mot_pos)->void
 {
	 //�˴�����ԭ���� ת����װ����Ľ���
	 //mot_pos[2] = -atan2(ee_position[2], ee_position[0]); //��ֱת���� ###�˴���ʱ����һ������
	 double theta0 = -atan2(ee_position[2], ee_position[0]);  //��ֱת����ת���ĽǶȣ�����Ҫת�����������
	 mot_pos[2] = 50 * 28 / 19 * theta0;  //��������ٱ�50��1���������� 28��19
	 double x0 = sqrt(ee_position[2] * ee_position[2] + ee_position[0] * ee_position[0]); //��������ƽ���xֵ
	 double y0 = ee_position[1]; //��������ƽ���yֵ

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
	 //mot_pos[0] = Hx - H_0x; //X�����Ƹ�
	 double deltaX = Hx - H_0x; //x�����Ƹ˱仯ֵ������Ҫת�����������ת�任ֵ
	 mot_pos[0] = 26 / 16 * deltaX / 0.0025; //����Ϊ2.5mm��ת����m��������������Ϊ26:16  ��������

	 

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
	 //mot_pos[1] = -(By - B_0y); //Y�����Ƹ�  �����ƶ��� ��Ҳ������
	 double deltaY = -(By - B_0y); //Y�����Ƹ˱仯ֵ������ת���������  �����ƶ��� ��Ҳ������
	 mot_pos[1] = 26 / 16 * deltaY / 0.0025; //����Ϊ2.5mm��ת����m��������������Ϊ26:16   ��������

 }

 //������任����ⷴ��
 //input�������ĩ�����꣨��������ϵ�£�
 //Body_P_Ground�ǵ�������������ϵ�ı��˴���ʼλ���غϣ���Ϊ4*4��λ��
 //Leg_P_Body����������ϵ��������ϵ�ı��
 //�����Ҫ����ĩ����������ϵ�µı��xyz_in_leg
 //Leg_xyz_ee =Leg_P_Body * Body_P_Ground * Ground_xyz_ee(input4*1)
 //A��B�б�ʾ�ı任������B��A�б�ʾ�ı仯����Ϊ�棬��Body_P_Ground * Ground_P_Body = I
 //inverseLeg�����Ǵ��루ĩ���ڵ����xyz���꣬�����ڵ�������꣬�������ĵط�������Ƹ˱仯ֵ����



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

	 double xyz_in_leg_l1[18] = { 0 }; //ÿ����ĩ���ڸ���������ϵ�µı�д��һ��������
	 //�ú��������루λ�˱仯����һ�����꣬���ת���������3*1��
	 aris::dynamic::s_pp2pp(real_pm_l1, ground_xyz_ee + 0 * 3, xyz_in_leg_l1 + 0 * 3); //l1
	 aris::dynamic::s_pp2pp(real_pm_l2, ground_xyz_ee + 1 * 3, xyz_in_leg_l1 + 1 * 3); //l2
	 aris::dynamic::s_pp2pp(real_pm_l3, ground_xyz_ee + 2 * 3, xyz_in_leg_l1 + 2 * 3); //l2
	 aris::dynamic::s_pp2pp(real_pm_l4, ground_xyz_ee + 3 * 3, xyz_in_leg_l1 + 3 * 3); //l2
	 aris::dynamic::s_pp2pp(real_pm_l5, ground_xyz_ee + 4 * 3, xyz_in_leg_l1 + 4 * 3); //l2
	 aris::dynamic::s_pp2pp(real_pm_l6, ground_xyz_ee + 5 * 3, xyz_in_leg_l1 + 5 * 3); //l2

	 //�õ�mot_pos
	 legInverseKinematics(xyz_in_leg_l1 + 0 * 3, input + 0 * 3);
	 legInverseKinematics(xyz_in_leg_l1 + 1 * 3, input + 1 * 3);
	 legInverseKinematics(xyz_in_leg_l1 + 2 * 3, input + 2 * 3);
	 legInverseKinematics(xyz_in_leg_l1 + 3 * 3, input + 3 * 3);
	 legInverseKinematics(xyz_in_leg_l1 + 4 * 3, input + 4 * 3);
	 legInverseKinematics(xyz_in_leg_l1 + 5 * 3, input + 5 * 3);
	 
	 return 0;
 }


