#ifndef KINEMATICS_H_
#define KINEMATICS_H_

//�Ȳ��˼�����//
//��λ��m
#define AC  0.185
#define CD  0.100
#define AG  0.100
#define DE  0.155
#define EN  0.220
#define DG  0.185
#define GF  0.060
#define GH  0.025
#define AJ  0.0905
#define LM  0.020
#define NC  0.475
#define BF  0.120
//NC = CD + DE + EN
//#define R   0.60477 //1��ĩ�˵�����ԭ���ƽ��ͶӰ���룬��ĩ����ת�İ뾶R


//x,y����ĳ�ʼֵ(ע������������֮��ľ���)
#define H_0x -0.002
#define B_0y 0.044

//���峤������ã���λm
//����ԭ�㽨�����µװ���ƽ�����ģ�"""������Adams����ϵһ��"""
#define FRONTX 0.300  //ǰ�󶥵���������ĵľ��룬X����
#define MIDX   0.150 //�м䶥����������ĵľ��룬X����
#define MIDZ   0.25981 //�м䶥����������ĵľ��룬Z����
#define HEIGHT 0.39291 //����ԭ������ڵ���߶ȣ� y���򣨸߶ȷ��򣩼�ĩ�˵�����ԭ��ľ���
#define AO_Y   0.03200 //A�㵽ԭ���Y����ľ���
#define AO_X   0.34800 //A�㵽ԭ���X����ľ���
#define EE1_X  0.60477 //1��ĩ�˵�����ԭ��X�������
#define EE6_X  0.30238 //6��ĩ�˵�����ԭ��X�������
#define EE6_Z  0.52374 //6��ĩ�˵�����ԭ��Z�������
#define PA_Y   0.032   //PA�������_Y  P��������ϵԭ��
#define PA_X   0.048   //PA�������_X


//��������ϵ��װ����Բ���غ�
//��ʼʱ���ڵ�������ϵ�µ�λ��
//����x������ĵ㿪ʼ����ʱ����ת����123456  ����ֱ�Ϊxyz
static double foot_position_start_point[18] = {
	EE1_X,		-HEIGHT,		0,
	EE6_X,		-HEIGHT,		-EE6_Z,
   -EE6_X,		-HEIGHT,		-EE6_Z,
   -EE1_X,		-HEIGHT,		0,
   -EE6_X,      -HEIGHT,		EE6_Z,
	EE6_X,		-HEIGHT,		EE6_Z };

//��ʼ��������ϵ���������ϵ�غ�
static double body_position_start_point[16] = {
	1,0,0,0,
	0,1,0,0,
	0,0,1,0,
	0,0,0,1 };

/**********��������**********/
auto inverseLeg(double *Ground_xyz_ee, double *Ground_P_Body, double *input)->int;

#endif

