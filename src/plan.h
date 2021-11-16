#ifndef PLAN_H_
#define PLAN_H_

//T型曲线
//产生0->1的梯形曲线，可根据输入的加速度和速度判断曲线是梯形还是三角形
/*
参数定义
Tc:生成曲线所需的周期时间，由速度和加速度计算
v:速度，由用户输入，构造函数初始化
a:加速度，用户输入，构造函数初始化
ta:加速度所需时间，由速度和加速度计算获得
*/
class TCurve
{
private:
	double Tc_;
	double v_;
	double a_;
	double ta_;
public:
	auto getTCurve(int count)->double; //输入count,输出s=0->1
	auto getCurveParam()->void; //根据用户输入的v,a,计算Tc，ta
	auto getTc()->double { return Tc_; };
	TCurve(double a, double v) { a_ = a; v_ = v; } //这是构造函数初始化的写法之一
	~TCurve() {} //析构函数
};

//传入,a,v,d(需要移动的距离)
class TCurve2
{
private:
	double d_;
	double Tc_;
	double v_;
	double a_;
	double ta_;
public:
	auto getTCurve(int count)->double; //输入count,输出s=0->1
	auto getCurveParam()->void; //根据用户输入的v,a,计算Tc，ta
	auto getTc()->double { return Tc_; };
	auto getta()->double { return ta_; };
	auto getv()->double { return v_; };
	auto geta()->double { return a_; };
	TCurve2(double a, double v, double d) { a_ = a; v_ = v; d_ = d; } //这是构造函数初始化的写法之一
	~TCurve2() {} //析构函数
};

//椭圆轨迹曲线
//功能：规划末端执行椭圆轨迹曲线，在Tc时间内，x方向从a0->a1,y方向b0->b1->b0，z方向c0->c1
/*
参数定义
a0，b0，c0：末端初始位置x，y，z值,默认初始位置
a1，c1：末端指定位置x，z值，由用户输入，构造函数初始化
b1：末端移动过程中最高位置y值，由用户输入，构造函数初始化
a:x方向步长，由用户输入，构造函数初始化
b:y方向步高，由用户输入，构造函数初始化
c:z方向步长，由用户输入，构造函数初始化
x,y,z：t时刻时对应x,y,z的位置+
s:：梯形曲线 0->1的一个数，这个数的导数是梯形曲线
*/


/*
目前设置的初始位置下
1腿末端到身体坐标系原点，dx=0.60477，dz=0.39291,  dy=0 ，故dx即为半径r
考虑三角步态完全对称的情况下，每步x方向步长Sx为初始末端到腿坐标系（建立在转轴处）原点距离的平面投影，Sx=0.30477，这个数目前看来可能不太合理，后面还需验证
这种步态下，2356腿投影平面来看末端轨迹为圆弧，转过60°，在增添Z方向的轨迹，即可生成椭圆轨迹，目前暂定y方向抬起的最高高度为0.1
*/

class EllipseTrajectory
{
private:
	double x_;
	double y_;
	double z_;
	double a_;
	double b_;
	double c_;
	TCurve s_;

public:
	auto getEllipseTrajectory(int count)->void;
	auto get_x()->double { return x_; };
	auto get_y()->double { return y_; };
	auto get_z()->double { return z_; };
	auto get_a()->double { return a_; };
	auto get_b()->double { return b_; };
	auto get_c()->double { return c_; };
	auto get_s()->TCurve { return s_; };
	EllipseTrajectory(double a, double b, double c, TCurve& s) :a_(a), b_(b), c_(c), s_(s), x_(0), y_(0), z_(0) {}; //构造函数初始化
	~EllipseTrajectory() {}           
};

class Gait
{
public:
	EllipseTrajectory E;

	~Gait() {};
};

/*功能：生成机器人绕xyz三个轴旋转的角度轨迹
   ##参数定义##
double roll_angle_x_：绕x轴旋转的角度，由用户输入，构造函数初始化
double yaw_angle_y_：绕y轴旋转的角度，由用户输入，构造函数初始化
double pitch_angle_z_：绕z轴旋转的角度，由用户输入，构造函数初始化
double pitch_：在t时刻的俯仰角  绕x轴旋转
double roll_：在t时刻的横滚角   绕z轴旋转
double yaw_：在t时刻的偏航角    绕y轴旋转
TCurve b_r_s_:梯形曲线
s:梯形曲线
*/
class BodyPose
{
private:
	double roll_angle_x_;
	double yaw_angle_y_;
	double pitch_angle_z_;
	double pitch_;
	double roll_;
	double yaw_;
	TCurve b_r_s_;

public:
	auto getBodyRotationTrajectory(int count)->void;
	auto getCurrentRoll()->double { return roll_; };
	auto getCurrentPitch()->double { return pitch_; };
	auto getCurrentYaw()->double { return yaw_; };
	auto getRollTotalAngle()->double { return roll_angle_x_; };
	auto getPitchTotalAngle()->double { return pitch_angle_z_; };
	auto getYawTotalAngle()->double { return yaw_angle_y_; };
	auto getTcurve()->TCurve { return b_r_s_; };
	//构造函数初始化列表方式
	BodyPose(double roll, double yaw, double pitch, TCurve s) : roll_angle_x_(roll), yaw_angle_y_(yaw), pitch_angle_z_(pitch), b_r_s_(s), pitch_(0), yaw_(0), roll_(0) {} 
	~BodyPose() {};
};

/*************************函数声明/*************************/
auto tripodPlan(int n, int count, EllipseTrajectory* Ellipse, double* input)->int;
auto turnPlanTripod(int n, int count, EllipseTrajectory* Ellipse, BodyPose* body_pose_param, double* input)->int;
auto tetrapodPlan(int n, int count, EllipseTrajectory* Ellipse, double* input)->int;
#endif