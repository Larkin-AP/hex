#ifndef KINEMATICS_H_
#define KINEMATICS_H_

//腿部杆件长度//
//单位：m
#define AC  0.185
#define CD  0.100
#define AG  0.100
#define DE  0.39224
#define DG  0.185
#define GF  0.060
#define GH  0.025
#define AJ  0.0905
#define LM  0.020
#define BF  0.120

const double EC = CD+DE;




//x,y方向的初始值(注意是哪两个面之间的距离)？？
//H和B点初始位置的坐标值，在以A为原点的坐标系
//#define H_0x 0.00914
//#define B_0y 0.05336

//#define H_0x 0.007105
//#define B_0y 0.0447558

//这个参数是刚好触发传感器的Hx和By坐标
#define H_0x -0.020
#define B_0y 0.069





//身体长宽高设置，单位m
//坐标原点建立在下底板下平面中心，"""方向与Adams坐标系一致"""
#define FRONTX 0.335  //前后顶点相对于中心的距离，X方向
#define MIDX   0.1675 //中间顶点相对于中心的距离，X方向
#define MIDZ   0.29012 //中间顶点相对于中心的距离，Z方向
#define HEIGHT 0.37252 //坐标原点相对于地面高度， y方向（高度方向）即末端到坐标原点的距离
#define AO_Y   0.03350 //A点到原点的Y方向的距离
#define AO_X   0.34585 //A点到原点的X方向的距离
#define EE1_X  0.68249 //1腿末端到坐标原点X方向距离
#define EE6_X  0.34125 //6腿末端到坐标原点X方向距离
#define EE6_Z  0.59105 //6腿末端到坐标原点Z方向距离
#define PA_Y   0.032   //PA距离分量_Y  P是腿坐标系原点
#define PA_X   0.048   //PA距离分量_X


//地面坐标系与底板底面圆心重合
//初始时脚在地面坐标系下的位置
//按照x正方向的点开始，逆时针旋转排序123456  坐标分别为xyz
//static double foot_position_start_point[18] = {
//    EE1_X,		-HEIGHT,		0,
//    EE6_X,		-HEIGHT,		-EE6_Z,
//   -EE6_X,		-HEIGHT,		-EE6_Z,
//   -EE1_X,		-HEIGHT,		0,
//   -EE6_X,      -HEIGHT,		EE6_Z,
//    EE6_X,		-HEIGHT,		EE6_Z };

//初始身体坐标系与地面坐标系重合
//double body_position_start_point[16];
//= {
//    1,0,0,0,
//    0,1,0,0,
//    0,0,1,0,
//    0,0,0,1 };

//当身体需要转动加移动时，需要记录改变当前身体的位置
extern double foot_position_start_point[18];
extern double body_position_start_point[16];
extern double body_related_world[16];
extern double foot_position_related_body[18];
extern double body_position_related_body[16];


extern double ans[2];







/**********函数声明**********/
auto inverseLeg(double *Ground_xyz_ee, double *Ground_P_Body, double *input)->int;
auto legInverseKinematics(double *ee_position, double *mot_pos)->void;//用于运动规划
auto legForwardKinematics(double *mot, double *ee_position)->void;
auto legInverseKinematics2(double *ee_position, double *mot_pos,double *ans)->void;//用于setInitPos，计算初始设立末端位置的电机值，不参与运动规划,ans[2] = {Hx,By}
auto calHxAndBy(double *mot, double *ans)->void; //用于获取设立初值后，计算获得Hx和By

#endif

