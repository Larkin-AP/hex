#ifndef KINEMATICS_H_
#define KINEMATICS_H_

//腿部杆件长度//
//单位：m
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
//#define R   0.60477 //1腿末端到身体原点的平面投影距离，即末端旋转的半径R


//x,y方向的初始值(注意是那两个面之间的距离)
#define H_0x -0.002
#define B_0y 0.044

//身体长宽高设置，单位m
//坐标原点建立在下底板下平面中心，"""方向与Adams坐标系一致"""
#define FRONTX 0.300  //前后顶点相对于中心的距离，X方向
#define MIDX   0.150 //中间顶点相对于中心的距离，X方向
#define MIDZ   0.25981 //中间顶点相对于中心的距离，Z方向
#define HEIGHT 0.39291 //坐标原点相对于地面高度， y方向（高度方向）即末端到坐标原点的距离
#define AO_Y   0.03200 //A点到原点的Y方向的距离
#define AO_X   0.34800 //A点到原点的X方向的距离
#define EE1_X  0.60477 //1腿末端到坐标原点X方向距离
#define EE6_X  0.30238 //6腿末端到坐标原点X方向距离
#define EE6_Z  0.52374 //6腿末端到坐标原点Z方向距离
#define PA_Y   0.032   //PA距离分量_Y  P是腿坐标系原点
#define PA_X   0.048   //PA距离分量_X


//地面坐标系与底板底面圆心重合
//初始时脚在地面坐标系下的位置
//按照x正方向的点开始，逆时针旋转排序123456  坐标分别为xyz
static double foot_position_start_point[18] = {
	EE1_X,		-HEIGHT,		0,
	EE6_X,		-HEIGHT,		-EE6_Z,
   -EE6_X,		-HEIGHT,		-EE6_Z,
   -EE1_X,		-HEIGHT,		0,
   -EE6_X,      -HEIGHT,		EE6_Z,
	EE6_X,		-HEIGHT,		EE6_Z };

//初始身体坐标系与地面坐标系重合
static double body_position_start_point[16] = {
	1,0,0,0,
	0,1,0,0,
	0,0,1,0,
	0,0,0,1 };

/**********函数声明**********/
auto inverseLeg(double *Ground_xyz_ee, double *Ground_P_Body, double *input)->int;

#endif

