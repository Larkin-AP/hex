#include<cmath>
#include<iostream>
#include<aris.hpp>
#include"kinematics.h"
#include"plan.h"

using namespace std;

extern double file_current_leg[18];
extern double file_current_body[16];
extern double PI;
//-------------------------------------------------------��������----------------------------------------------------//

//������������0->1
//���룺ʱ�䣬ÿ�������һ��
//�������ǰʱ��s��ֵ
auto TCurve::getTCurve(int count)->double
{
	//double ta = p.ta_;
	//double a = p.a_;
	//double v = p.v_;
	//double T_c = p.Tc_;
	int t = count + 1;
	double s = 0;

	if (2 * ta_ == Tc_)   //����������
	{
		if (t < ta_ * 1000)
		{
			s = 0.5 * a_ * t * t / 1000.0 / 1000.0;
		}
		else
		{
			s = 0.5 * a_ * ta_ * ta_ + 0.5 * (t / 1000.0 - ta_) * (2 * v_ - a_ * (t / 1000.0 - ta_));
		}
	}
	else    //��������
	{
		if (t < ta_ * 1000)
		{
			s = 0.5 * a_ * t * t / 1000.0 / 1000.0;
		}
		else if (t >= ta_ * 1000 && t < (Tc_ * 1000 - ta_ * 1000))
		{
			s = v_ * t / 1000 - v_ * v_ / 2.0 / a_;
		}
		else
		{
			s = (2 * a_ * v_ * Tc_ - 2 * v_ * v_ - a_ * a_ * (t / 1000.0 - Tc_) * (t / 1000.0 - Tc_)) / (2 * a_);
		}
	}
	//std::cout << s << std::endl;
	return s;
}

//�����������ߵĲ������ɳ�Ա������ʼ������Ӧ��������ɹ��캯����ʼ��
auto TCurve::getCurveParam()->void
{
	if (v_ * v_ / a_ <= 1) //��������
	{
		this->Tc_ = (a_ + v_ * v_) / v_ / a_;
		this->a_ = a_;
		this->v_ = v_;
	}
	else
	{
		//���ٶȼ��㣬��ʱ�����ļ��ٶȲ�������
		this->Tc_ = 2.0 / v_;  //����������
		this->a_ = v_ * v_;
		this->v_ = v_;
	}
	this->ta_ = v_ / a_;
}



//---------------------------------------------------------��Բ�켣---------------------------------------------------//

//������Բ�켣����Tcʱ����  x����0->a/2;y����0->b->0;z����0->c/2����Ӧ��������ɹ��캯����ʼ����
//��������Ϊ��Բ�ĳ���Ͷ��ᣬ��������ط�xz���2
auto EllipseTrajectory::getEllipseTrajectory(int count)->void
{
	x_ = a_ * (1 + cos(PI - PI * s_.getTCurve(count))) / 2.0;
	y_ = b_ * sin(PI - PI * s_.getTCurve(count));
	z_ = c_ * (1 + cos(PI - PI * s_.getTCurve(count))) / 2.0;
	//std::cout << y << std::endl;
}



//-----------------------------------------------------������ת�Ƕȹ켣----------------------------------------------//

//����������xyz������ת�ĽǶȹ켣0->theta  �Ƕ�Ϊ����
auto BodyPose::getBodyRotationTrajectory(int count)->void
{
	pitch_ = pitch_angle_z_ * PI * b_r_s_.getTCurve(count) / 180.0;  //pitch  ת��Ϊ���� ������������������
	roll_ = roll_angle_x_ * PI * b_r_s_.getTCurve(count) / 180.0;
	yaw_ = yaw_angle_y_ * PI * b_r_s_.getTCurve(count) / 180.0;

	//std::cout << b_r_s_.getTCurve << std::endl;
	//std::cout << pitch_ << std::endl;
}



//---------------------------------------------------�滮��---------------------------------------------//

///����ڵѿ�������ϵ�µĹ滮��ÿִ����һ���������߼�¼һ������
///count=e_2 ��0 �� Tc ѭ��  
///�жϵ�ǰ������һ��,����һ��e1��1

//���ǲ�̬
//��ǰ�ŵ�λ�� = ��һ���ŵ�λ�� + ��λ������
//#ע�⣺Ŀǰֻ������ƽ������
//e_1������¼��ǰ�ߵ��ڼ���������Ҫ��n����count��һ�����ȵļ�ʱ���ʲ��ܰ��ܵ�ʱ�Ӵ��룬Ҫ����
auto planLegTripod(int e_1, int n, double* current_leg, int count, EllipseTrajectory* Ellipse)->void
{
	if (count == 0)//��ʼ���ŵ�λ��
	{
		for (int i = 0; i < 18; ++i)
		{
			current_leg[i] = foot_position_start_point[i];
		}
	}

	Ellipse->getEllipseTrajectory(count);

	if (e_1 % 2 == 0)  //ż��135���ȣ�246ͣ
	{
		if (e_1 == 0)   //���ٶ�
		{
			//�滮leg1
			current_leg[0] = foot_position_start_point[0] + Ellipse->get_x() / 2;
			current_leg[1] = foot_position_start_point[1] + Ellipse->get_y();
			current_leg[2] = foot_position_start_point[2] + Ellipse->get_z() / 2;
			//�滮leg3
			current_leg[6] = foot_position_start_point[6] + Ellipse->get_x() / 2;
			current_leg[7] = foot_position_start_point[7] + Ellipse->get_y();
			current_leg[8] = foot_position_start_point[8] + Ellipse->get_z() / 2;
			//leg5
			current_leg[12] = foot_position_start_point[12] + Ellipse->get_x() / 2;
			current_leg[13] = foot_position_start_point[13] + Ellipse->get_y();
			current_leg[14] = foot_position_start_point[14] + Ellipse->get_z() / 2;
		}
		else
		{
			//�滮leg1
			current_leg[0] = foot_position_start_point[0] + Ellipse->get_x();
			current_leg[1] = foot_position_start_point[1] + Ellipse->get_y();
			current_leg[2] = foot_position_start_point[2] + Ellipse->get_z();


			//�滮leg3
			current_leg[6] = foot_position_start_point[6] + Ellipse->get_x();
			current_leg[7] = foot_position_start_point[7] + Ellipse->get_y();
			current_leg[8] = foot_position_start_point[8] + Ellipse->get_z();
			//leg5
			current_leg[12] = foot_position_start_point[12] + Ellipse->get_x();
			current_leg[13] = foot_position_start_point[13] + Ellipse->get_y();
			current_leg[14] = foot_position_start_point[14] + Ellipse->get_z();
		}
	}
	else if (e_1 % 2 == 1)  //����24����13ͣ
	{
		if (e_1 == (2 * n - 1))//���ٶ�
		{
			//�滮leg2
			current_leg[3] = foot_position_start_point[3] + Ellipse->get_x() / 2;
			current_leg[4] = foot_position_start_point[4] + Ellipse->get_y();
			current_leg[5] = foot_position_start_point[5] + Ellipse->get_z() / 2;
			//�滮leg4
			current_leg[9] = foot_position_start_point[9] + Ellipse->get_x() / 2;
			current_leg[10] = foot_position_start_point[10] + Ellipse->get_y();
			current_leg[11] = foot_position_start_point[11] + Ellipse->get_z() / 2;
			//leg6
			current_leg[15] = foot_position_start_point[15] + Ellipse->get_x() / 2;
			current_leg[16] = foot_position_start_point[16] + Ellipse->get_y();
			current_leg[17] = foot_position_start_point[17] + Ellipse->get_z() / 2;
		}
		else
		{
			//�滮leg2
			current_leg[3] = foot_position_start_point[3] + Ellipse->get_x();
			current_leg[4] = foot_position_start_point[4] + Ellipse->get_y();
			current_leg[5] = foot_position_start_point[5] + Ellipse->get_z();
			//�滮leg4
			current_leg[9] = foot_position_start_point[9] + Ellipse->get_x();
			current_leg[10] = foot_position_start_point[10] + Ellipse->get_y();
			current_leg[11] = foot_position_start_point[11] + Ellipse->get_z();
			//leg6
			current_leg[15] = foot_position_start_point[15] + Ellipse->get_x();
			current_leg[16] = foot_position_start_point[16] + Ellipse->get_y();
			current_leg[17] = foot_position_start_point[17] + Ellipse->get_z();
		}
	}

	if (count + 1 == std::floor(Ellipse->get_s().getTc() * 1000)) //floor ���� ����ȡ��
	{
		for (int i = 0; i < 18; ++i)
		{
			foot_position_start_point[i] = current_leg[i];
		}
	}
}

//���ǲ�̬��ԭ����ת
//�滮����ֱ����̧��Ȼ�����ת���󣿣�
//#ע�⣺Ŀǰֻ������ƽ��
auto planLegTripodTurn(int e_1, double* current_leg, int count, EllipseTrajectory* Ellipse, BodyPose* body_pose_param)->void
{

	//count ��0 �� Tc ѭ��  //�жϵ�ǰ������һ��,����һ��e1��1
	
	if (count == 0)//��ʼ���ŵ�λ��
	{
		for (int i = 0; i < 18; ++i)
		{
			current_leg[i] = foot_position_start_point[i];
		}
	}
	Ellipse->getEllipseTrajectory(count);

	double temp_xyz_in_ground[18] = { 0 };
	static double yaw = 0;
	//ÿ���������߿�ʼʱ��ȡ֮ǰ��ֵ
	if (count == 0)
	{
		yaw = 0;
	}

	body_pose_param->getBodyRotationTrajectory(count); //�õ���count�仯��rpy
	yaw = body_pose_param->getCurrentYaw(); //�õ���ǰʱ�̵�yaw

	double R_y[16] = {
						 std::cos(yaw), 0, std::sin(yaw), 0,
								0, 1,        0, 0,
						-std::sin(yaw), 0, std::cos(yaw), 0,
								0, 0,        0, 1
	};

	//�������滮��������̧��
	if (e_1 % 2 == 0)  //ż��135���ȣ�246ͣ
	{
		//�滮leg1
		temp_xyz_in_ground[0] = foot_position_start_point[0] + Ellipse->get_x();
		temp_xyz_in_ground[1] = foot_position_start_point[1] + Ellipse->get_y();
		temp_xyz_in_ground[2] = foot_position_start_point[2] + Ellipse->get_z();


		//�滮leg3
		temp_xyz_in_ground[6] = foot_position_start_point[6] + Ellipse->get_x();
		temp_xyz_in_ground[7] = foot_position_start_point[7] + Ellipse->get_y();
		temp_xyz_in_ground[8] = foot_position_start_point[8] + Ellipse->get_z();

		//�滮leg5
		temp_xyz_in_ground[12] = foot_position_start_point[12] + Ellipse->get_x();
		temp_xyz_in_ground[13] = foot_position_start_point[13] + Ellipse->get_y();
		temp_xyz_in_ground[14] = foot_position_start_point[14] + Ellipse->get_z();

		aris::dynamic::s_pp2pp(R_y, temp_xyz_in_ground + 0 * 3, current_leg + 0 * 3);
		aris::dynamic::s_pp2pp(R_y, temp_xyz_in_ground + 4 * 3, current_leg + 4 * 3);

	}
	else if (e_1 % 2 == 1)  //����246����135ͣ
	{
		//�滮leg2
		temp_xyz_in_ground[3] = foot_position_start_point[3] + Ellipse->get_x();
		temp_xyz_in_ground[4] = foot_position_start_point[4] + Ellipse->get_y();
		temp_xyz_in_ground[5] = foot_position_start_point[5] + Ellipse->get_z();
		//�滮leg4
		temp_xyz_in_ground[9] = foot_position_start_point[9] + Ellipse->get_x();
		temp_xyz_in_ground[10] = foot_position_start_point[10] + Ellipse->get_y();
		temp_xyz_in_ground[11] = foot_position_start_point[11] + Ellipse->get_z();
		//�滮leg6
		temp_xyz_in_ground[15] = foot_position_start_point[15] + Ellipse->get_x();
		temp_xyz_in_ground[16] = foot_position_start_point[16] + Ellipse->get_y();
		temp_xyz_in_ground[17] = foot_position_start_point[17] + Ellipse->get_z();

		aris::dynamic::s_pp2pp(R_y, temp_xyz_in_ground + 1 * 3, current_leg + 1 * 3);
		aris::dynamic::s_pp2pp(R_y, temp_xyz_in_ground + 3 * 3, current_leg + 3 * 3);
		aris::dynamic::s_pp2pp(R_y, temp_xyz_in_ground + 5 * 3, current_leg + 5 * 3);
	}


	//ÿ���һ���������ߺ��¼һ�νŵ�λ��
	if (count + 1 == floor(Ellipse->get_s().getTc() * 1000))
	{
		for (int i = 0; i < 18; ++i)
		{
			foot_position_start_point[i] = current_leg[i];
		}
	}
}

//���㲽̬
//��ǰ�ŵ�λ�� = ��һ���ŵ�λ�� + ��λ������
//#ע�⣺Ŀǰֻ������ƽ������
auto planLegTetrapod(int e_1, int n, double* current_leg, int count, EllipseTrajectory* Ellipse)->void
{
	if (count == 0)//��ʼ���ŵ�λ��
	{
		for (int i = 0; i < 18; ++i)
		{
			current_leg[i] = foot_position_start_point[i];
		}
	}
	Ellipse->getEllipseTrajectory(count);


	//14  25  36  
	if (e_1  % 3 == 0)  //��14��
	{
		if (e_1 == 0)   //���ٶ�
		{
			//�滮leg1
			current_leg[0] = foot_position_start_point[0] + Ellipse->get_x() / 2;
			current_leg[1] = foot_position_start_point[1] + Ellipse->get_y();
			current_leg[2] = foot_position_start_point[2] + Ellipse->get_z() / 2;

			//�滮leg4
			current_leg[9] = foot_position_start_point[9] + Ellipse->get_x() / 2;
			current_leg[10] = foot_position_start_point[10] + Ellipse->get_y();
			current_leg[11] = foot_position_start_point[11] + Ellipse->get_z() / 2;
		}
		else if (e_1 == 3 * n - 3) //���ٶ�
		{
			//�滮leg1  
			current_leg[0] = foot_position_start_point[0] + Ellipse->get_x() / 2;
			current_leg[1] = foot_position_start_point[1] + Ellipse->get_y();
			current_leg[2] = foot_position_start_point[2] + Ellipse->get_z() / 2;

			//�滮leg4
			current_leg[9] = foot_position_start_point[9] + Ellipse->get_x() / 2;
			current_leg[10] = foot_position_start_point[10] + Ellipse->get_y();
			current_leg[11] = foot_position_start_point[11] + Ellipse->get_z() / 2;
		}
		else  //���ٶ�
		{
			//�滮leg1 
			current_leg[0] = foot_position_start_point[0] + Ellipse->get_x();
			current_leg[1] = foot_position_start_point[1] + Ellipse->get_y();
			current_leg[2] = foot_position_start_point[2] + Ellipse->get_z();

			//�滮leg4 
			current_leg[9] = foot_position_start_point[9] + Ellipse->get_x();
			current_leg[10] = foot_position_start_point[10] + Ellipse->get_y();
			current_leg[11] = foot_position_start_point[11] + Ellipse->get_z();
		}
	}
	else if (e_1  % 3 == 1)  //��25��
	{
		if (e_1 == 1)//���ٶ�
		{
			//�滮leg2
			current_leg[3] = foot_position_start_point[3] + Ellipse->get_x() / 2;
			current_leg[4] = foot_position_start_point[4] + Ellipse->get_y();
			current_leg[5] = foot_position_start_point[5] + Ellipse->get_z() / 2;
			//�滮leg5
			current_leg[12] = foot_position_start_point[12] + Ellipse->get_x() / 2;
			current_leg[13] = foot_position_start_point[13] + Ellipse->get_y();
			current_leg[14] = foot_position_start_point[14] + Ellipse->get_z() / 2;
		}
		else if (e_1 == 3 * n - 2)  //���ٶ�
		{
			//�滮leg2
			current_leg[3] = foot_position_start_point[3] + Ellipse->get_x() / 2;
			current_leg[4] = foot_position_start_point[4] + Ellipse->get_y();
			current_leg[5] = foot_position_start_point[5] + Ellipse->get_z() / 2;
			//�滮leg5
			current_leg[12] = foot_position_start_point[12] + Ellipse->get_x() / 2;
			current_leg[13] = foot_position_start_point[13] + Ellipse->get_y();
			current_leg[14] = foot_position_start_point[14] + Ellipse->get_z() / 2;
		}
		else
		{
			//�滮leg2
			current_leg[3] = foot_position_start_point[3] + Ellipse->get_x();
			current_leg[4] = foot_position_start_point[4] + Ellipse->get_y();
			current_leg[5] = foot_position_start_point[5] + Ellipse->get_z();
			//�滮leg5
			current_leg[12] = foot_position_start_point[12] + Ellipse->get_x();
			current_leg[13] = foot_position_start_point[13] + Ellipse->get_y();
			current_leg[14] = foot_position_start_point[14] + Ellipse->get_z();
		}
	}
	else if (e_1  % 3 == 2)  //��36��
	{
		if (e_1 == 2)//���ٶ�
		{
			//�滮leg3
			current_leg[6] = foot_position_start_point[6] + Ellipse->get_x() / 2;
			current_leg[7] = foot_position_start_point[7] + Ellipse->get_y();
			current_leg[8] = foot_position_start_point[8] + Ellipse->get_z() / 2;

			//�滮leg6
			current_leg[15] = foot_position_start_point[15] + Ellipse->get_x() / 2;
			current_leg[16] = foot_position_start_point[16] + Ellipse->get_y();
			current_leg[17] = foot_position_start_point[17] + Ellipse->get_z() / 2;
		}
		else if (e_1 == 3 * n - 1) //���ٶ�
		{
			//�滮leg3
			current_leg[6] = foot_position_start_point[6] + Ellipse->get_x() / 2;
			current_leg[7] = foot_position_start_point[7] + Ellipse->get_y();
			current_leg[8] = foot_position_start_point[8] + Ellipse->get_z() / 2;

			//�滮leg6
			current_leg[15] = foot_position_start_point[15] + Ellipse->get_x() / 2;
			current_leg[16] = foot_position_start_point[16] + Ellipse->get_y();
			current_leg[17] = foot_position_start_point[17] + Ellipse->get_z() / 2;
		}
		else  //���ٶ�
		{
			//�滮leg3
			current_leg[6] = foot_position_start_point[6] + Ellipse->get_x();
			current_leg[7] = foot_position_start_point[7] + Ellipse->get_y();
			current_leg[8] = foot_position_start_point[8] + Ellipse->get_z();

			//�滮leg6
			current_leg[15] = foot_position_start_point[15] + Ellipse->get_x();
			current_leg[16] = foot_position_start_point[16] + Ellipse->get_y();
			current_leg[17] = foot_position_start_point[17] + Ellipse->get_z();
		}
	}


	if (count == floor(Ellipse->get_s().getTc() * 1000) - 1)
	{
		for (int i = 0; i < 18; ++i)
		{
			foot_position_start_point[i] = current_leg[i];
		}
	}
}


//------------------------------------------------�滮����----------------------------------------------//

//���������ڹ滮��������������ǲ�̬�������λ�ù켣��������ת����̬�任��
//��ǰ�����λ�� = ��һ�������λ�� + ����λ������
//ÿ������һ�������ǣ������λ�ø���һ��
//#ע�⣺Ŀǰֻ������ƽ������
auto planBodyTransformTripod(int e_1, int n, double* current_body, int count, EllipseTrajectory* Ellipse)->void
{
	int per_step_count = Ellipse->get_s().getTc() * 1000;
	if (count == 0) //��ʼ������λ��
	{
		for (int i = 0; i < 16; ++i)
		{
			current_body[i] = body_position_start_point[i];
		}
	}

	if (e_1 == 0)   //���ٶ�
	{
		//�滮����
		current_body[3] = body_position_start_point[3] + Ellipse->get_a() * count * count / (4.0 * per_step_count * per_step_count);
		current_body[7] = body_position_start_point[7];
		current_body[11] = body_position_start_point[11] + Ellipse->get_c() * count * count / (4.0 * per_step_count * per_step_count);
	}
	else if (e_1 == (2 * n - 1))//���ٶ�
	{

		//�滮����
		int t = (2 * n - 1) * per_step_count + per_step_count;
		current_body[3] = body_position_start_point[3] + +0 - Ellipse->get_a() * (count - t) * (count - t) / (4.0 * per_step_count * per_step_count) + Ellipse->get_a() * n - Ellipse->get_a() / 2.0;//n * a
		current_body[7] = body_position_start_point[7];
		current_body[11] = body_position_start_point[11] + 0 - Ellipse->get_c() * (count - t) * (count - t) / (4.0 * per_step_count * per_step_count) + Ellipse->get_c() * n - Ellipse->get_c() / 2.0;
	}
	else //���ٶ�
	{
		//�滮����
		current_body[3] = body_position_start_point[3] + Ellipse->get_a() / 4.0 + Ellipse->get_a() * (count - per_step_count) / per_step_count / 2;//�ٶ�Ϊ75mm/s  ÿ�����per_step_count
		current_body[7] = body_position_start_point[7];
		current_body[11] = body_position_start_point[11] + Ellipse->get_c() / 4.0 + Ellipse->get_c() * (count - per_step_count) / per_step_count / 2;
	}
	//���������˶�Ϊ��������
	if (count + 1 >= 2 * n * per_step_count)
	{
		for (int i = 0; i < 16; ++i)
		{
			body_position_start_point[i] = current_body[i];
		}
	}
}

//���������ڹ滮���������ԭ����ת
//ÿһ����������ת�������Ƕȵ�һ�룬��tripod��̬��Ӧ
//count �� 0 -> count
auto planBodyTurn(int count, double* current_body, BodyPose* body_pose_param)->void
{
	double yaw = 0;

	//ÿ���������߿�ʼʱ��ȡ֮ǰ��ֵ
	if (count == 0)
	{
		//yaw = body_pose_start_yaw;
	}
	if (count == 0) //���ã�����ɾ�������㲻���Ƕ�
	{
		for (int i = 0; i < 16; ++i)
		{
			current_body[i] = body_position_start_point[i];
		}
	}
	body_pose_param->getBodyRotationTrajectory(count); //��body���õĽǶȲ����õ������Ƶ���ת�ǣ���ʱ�䣩

	yaw = body_pose_param->getCurrentYaw() / 2;

	double R_y[16] = {
						 std::cos(yaw), 0, std::sin(yaw), 0,
								0, 1,        0, 0,
						-std::sin(yaw), 0, std::cos(yaw), 0,
								0, 0,        0, 1
	};

	double tempy[16] = { 0 };

	aris::dynamic::s_pm_dot_pm(body_position_start_point, R_y, tempy); //������ˡ�tempy�õ�������ת�������λ��
	std::copy(tempy, tempy + 16, current_body); //copy���������ݵ��׵�ַ��β��ַ������Ŀ�ĵص��׵�ַ�� �õ�current_body

	//����ʱ����仯֮���ֵ
	if (count + 1 == std::floor(body_pose_param->getTcurve().getTc() * 1000)) //std::floor ����ȡ����
	{

		for (int i = 0; i < 16; i++)
		{
			body_position_start_point[i] = current_body[i];
		}
	}
}

//���������ڹ滮��������������㲽̬�������λ�ù켣��������ת����̬�任��
//��ǰ�����λ�� = ��һ�������λ�� + ����λ������
//ÿ������һ�������ǣ������λ�ø���һ��
//#ע�⣺Ŀǰֻ������ƽ������
auto planBodyTransformTetrapod(int e_1, int n, double* current_body, int count, EllipseTrajectory* Ellipse)->void
{
	int per_step_count = Ellipse->get_s().getTc() * 1000;
	if (count == 0) //��ʼ������λ��
	{
		for (int i = 0; i < 16; ++i)
		{
			current_body[i] = body_position_start_point[i];
		}
	}

	if (e_1 <= 2)   //���ٶ�
	{
		//�滮����
		current_body[3] = body_position_start_point[3] + Ellipse->get_a() * count * count / (18.0 * per_step_count * per_step_count);
		current_body[7] = body_position_start_point[7];
		current_body[11] = body_position_start_point[11] + Ellipse->get_c() * count * count / (18.0 * per_step_count * per_step_count);
	}
	else if (e_1 >= (3 * n - 3))//���ٶ�
	{

		//�滮����
		int t = 3 * n * per_step_count;
		current_body[3] = body_position_start_point[3] - Ellipse->get_a() * (count - t) * (count - t) / (18.0 * per_step_count * per_step_count) + Ellipse->get_a() * (n - 1);//n * a
		current_body[7] = body_position_start_point[7];
		current_body[11] = body_position_start_point[11] - Ellipse->get_c() * (count - t) * (count - t) / (18.0 * per_step_count * per_step_count) + Ellipse->get_c() * (n - 1);
	}
	else //���ٶ�
	{
		//�滮����
		current_body[3] = body_position_start_point[3] + Ellipse->get_a() / 2.0 + Ellipse->get_a() * (count -3* per_step_count) / per_step_count / 3;//�ٶ�Ϊ75mm/s  ÿ�����per_step_count
		current_body[7] = body_position_start_point[7];
		current_body[11] = body_position_start_point[11] + Ellipse->get_c() / 2.0 + Ellipse->get_c() * (count -3* per_step_count) / per_step_count / 3;
	}
	//���������˶�Ϊ��������
	if (count + 1 >= 3 * n * per_step_count)
	{
		for (int i = 0; i < 16; ++i)
		{
			body_position_start_point[i] = current_body[i];
		}
	}
}

//--------------------------------------------------��̬�滮---------------------------------------------------------//

//�������������ǲ�̬������ԭ��̤����ǰ�������ˡ����ơ����ơ�
//���в������ߺͲ��������û����롣��һ����ʱ�䣨�����߿����������û������������ߵ��ٶȺͼ��ٶ�ȷ��
//#ע�⣺��������ٶȺͼ��ٶȻ�û����
auto tripodPlan(int n, int count, EllipseTrajectory* Ellipse, double* input)->int
{

	int per_step_count = Ellipse->get_s().getTc() * 1000;

	static double current_leg_in_ground[18] = { 0 };
	static double current_body_in_ground[16] = { 0 };
	//�ж�����״̬
	int e_1 = count / per_step_count;  //�жϵ�ǰ������һ��,����һ��,e1��1
	int e_2 = count % per_step_count;  //��һ����������0->Tc ��count


	//�滮��
	//e_1���жϵ�ǰ�ߵ��ڼ�����e_2�ǵ�ǰTc�ڵ�count
	//������ȵ�Tc����һ��Tc
	planLegTripod(e_1, n, current_leg_in_ground, e_2, Ellipse);
	//�滮����
	planBodyTransformTripod(e_1, n, current_body_in_ground, count, Ellipse);

	//ģ�Ͳ���ʹ��
	for (int i = 0; i < 18; ++i)
	{

		file_current_leg[i] = current_leg_in_ground[i];
	}
	for (int i = 0; i < 16; ++i)
	{
		file_current_body[i] = current_body_in_ground[i];
	}
	inverseLeg(current_leg_in_ground, current_body_in_ground, input);

	return 2 * n * per_step_count - count - 1;
}

//�������������㲽̬������ԭ��̤����ǰ�������ˡ����ơ����ơ�
//���в������ߺͲ��������û����롣��һ����ʱ�䣨�����߿����������û������������ߵ��ٶȺͼ��ٶ�ȷ��
//#ע�⣺��������ٶȺͼ��ٶȻ�û����
auto tetrapodPlan(int n, int count, EllipseTrajectory* Ellipse, double* input)->int
{

	int per_step_count = Ellipse->get_s().getTc() * 1000;

	static double current_leg_in_ground[18] = { 0 };
	static double current_body_in_ground[16] = { 0 };
	//�ж�����״̬
	int e_1 = count / per_step_count;  //�жϵ�ǰ������һ��,����һ��,e1��1
	int e_2 = count % per_step_count;  //0->Tc count


	//�滮��
	planLegTetrapod(e_1, n, current_leg_in_ground, e_2, Ellipse);
	//�滮����
	planBodyTransformTetrapod(e_1, n, current_body_in_ground, count, Ellipse);

	//ģ�Ͳ���ʹ��
	for (int i = 0; i < 18; ++i)
	{
		file_current_leg[i] = current_leg_in_ground[i];
	}
	for (int i = 0; i < 16; ++i)
	{
		file_current_body[i] = current_body_in_ground[i];
	}
	inverseLeg(current_leg_in_ground, current_body_in_ground, input);

	return 3 * n * per_step_count - count - 1;
}


//�Խǲ�̬��ԭ����ת
auto turnPlanTripod(int n, int count, EllipseTrajectory* Ellipse, BodyPose* body_pose_param, double* input)->int
{

	int per_step_count = Ellipse->get_s().getTc() * 1000;

	static double current_leg_in_ground[18] = { 0 };
	static double current_body_in_ground[16] = { 0 };

	//�ж�����״̬
	int e_1 = count / per_step_count;  //�жϵ�ǰ������һ��,����һ��,e1��1
	int e_2 = count % per_step_count;  //0->Tc count

	//�滮��
	planLegTripodTurn(e_1, current_leg_in_ground, e_2, Ellipse, body_pose_param);
	//�滮����
	planBodyTurn(e_2, current_body_in_ground, body_pose_param);


	//ģ�Ͳ���ʹ��
	for (int i = 0; i < 18; ++i)
	{
		file_current_leg[i] = current_leg_in_ground[i];
	}
	for (int i = 0; i < 16; ++i)
	{
		file_current_body[i] = current_body_in_ground[i];
	}
	//ģ�Ͳ���ʹ��

	inverseLeg(current_leg_in_ground, current_body_in_ground, input);
	

	return  per_step_count * n * 2 - count - 1;
}






















