#ifndef PLAN_H_
#define PLAN_H_

//T������
//����0->1���������ߣ��ɸ�������ļ��ٶȺ��ٶ��ж����������λ���������
/*
��������
Tc:�����������������ʱ�䣬���ٶȺͼ��ٶȼ���
v:�ٶȣ����û����룬���캯����ʼ��
a:���ٶȣ��û����룬���캯����ʼ��
ta:���ٶ�����ʱ�䣬���ٶȺͼ��ٶȼ�����
*/
class TCurve
{
private:
	double Tc_;
	double v_;
	double a_;
	double ta_;
public:
	auto getTCurve(int count)->double; //����count,���s=0->1
	auto getCurveParam()->void; //�����û������v,a,����Tc��ta
	auto getTc()->double { return Tc_; };
	TCurve(double a, double v) { a_ = a; v_ = v; } //���ǹ��캯����ʼ����д��֮һ
	~TCurve() {} //��������
};

//��Բ�켣����
//���ܣ��滮ĩ��ִ����Բ�켣���ߣ���Tcʱ���ڣ�x�����a0->a1,y����b0->b1->b0��z����c0->c1
/*
��������
a0��b0��c0��ĩ�˳�ʼλ��x��y��zֵ,Ĭ�ϳ�ʼλ��
a1��c1��ĩ��ָ��λ��x��zֵ�����û����룬���캯����ʼ��
b1��ĩ���ƶ����������λ��yֵ�����û����룬���캯����ʼ��
a:x���򲽳������û����룬���캯����ʼ��
b:y���򲽸ߣ����û����룬���캯����ʼ��
c:z���򲽳������û����룬���캯����ʼ��
x,y,z��tʱ��ʱ��Ӧx,y,z��λ��+
s:���������� 0->1��һ������������ĵ�������������
*/


/*
Ŀǰ���õĳ�ʼλ����
1��ĩ�˵���������ϵԭ�㣬dx=0.60477��dz=0.39291,  dy=0 ����dx��Ϊ�뾶r
�������ǲ�̬��ȫ�ԳƵ�����£�ÿ��x���򲽳�SxΪ��ʼĩ�˵�������ϵ��������ת�ᴦ��ԭ������ƽ��ͶӰ��Sx=0.30477�������Ŀǰ�������ܲ�̫�������滹����֤
���ֲ�̬�£�2356��ͶӰƽ������ĩ�˹켣ΪԲ����ת��60�㣬������Z����Ĺ켣������������Բ�켣��Ŀǰ�ݶ�y����̧�����߸߶�Ϊ0.1
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
	EllipseTrajectory(double a, double b, double c, TCurve& s) :a_(a), b_(b), c_(c), s_(s), x_(0), y_(0), z_(0) {}; //���캯����ʼ��
	~EllipseTrajectory() {}           
};

class Gait
{
public:
	EllipseTrajectory E;

	~Gait() {};
};

/*���ܣ����ɻ�������xyz��������ת�ĽǶȹ켣
   ##��������##
double roll_angle_x_����x����ת�ĽǶȣ����û����룬���캯����ʼ��
double yaw_angle_y_����y����ת�ĽǶȣ����û����룬���캯����ʼ��
double pitch_angle_z_����z����ת�ĽǶȣ����û����룬���캯����ʼ��
double pitch_����tʱ�̵ĸ�����  ��x����ת
double roll_����tʱ�̵ĺ����   ��z����ת
double yaw_����tʱ�̵�ƫ����    ��y����ת
TCurve b_r_s_:��������
s:��������
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
	//���캯����ʼ���б�ʽ
	BodyPose(double roll, double yaw, double pitch, TCurve s) : roll_angle_x_(roll), yaw_angle_y_(yaw), pitch_angle_z_(pitch), b_r_s_(s), pitch_(0), yaw_(0), roll_(0) {} 
	~BodyPose() {};
};

/*************************��������/*************************/
auto tripodPlan(int n, int count, EllipseTrajectory* Ellipse, double* input)->int;
auto turnPlanTripod(int n, int count, EllipseTrajectory* Ellipse, BodyPose* body_pose_param, double* input)->int;
auto tetrapodPlan(int n, int count, EllipseTrajectory* Ellipse, double* input)->int;
#endif