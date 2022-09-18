#include<algorithm>
#include<array>
#include<stdlib.h>
#include<string>
#include<bitset>

#include"robot.h"
#include"plan.h"
#include"kinematics.h"

double input_angle[18] = { 0 };
double init_pos_angle[18] = { 0 };

//输出参数，模型曲线测试使用
double file_current_leg[18] = { 0 };
double file_current_body[16] = { 0 };
double time_test = 0;
extern double PI;

using namespace aris::dynamic;
using namespace aris::plan;
namespace robot
{



//----------------------------------以下为仿真区域------------------------------------------//


            //---------------------------cpp和Adams调试------------------------//

            //前进
            auto HexDynamicForwardTest::prepareNrt()->void
        	{

        	}
        	auto HexDynamicForwardTest::executeRT()->int
        	{
                //数值解和实际解xyr相差一个负号
                //如果要输出cmd文件，则不能创建储存文件，需要注释掉
                //if (count() == 1)this->master()->logFileRawName("eeTraj");    
                //if (count() == 1)this->master()->logFileRawName("inputTraj");
                //if (count() == 1)this->master()->logFileRawName("invInput"); //反解计算结果储存文件，即解析解
                //if (count() == 1)this->master()->logFileRawName("numInput"); //数值解储存文件
                //if (count() == 1)this->master()->logFileRawName("leg1MotorPos1"); //拿到第1条腿的电机位置
                //if (count() == 1)this->master()->logFileRawName("leg1EndTraj1"); //拿到第1条腿的末端位置，要得到腿坐标系下的

                //a为给机器人缓冲落地的时间设置
        		int ret = 0,a=500;
                //末端为六个末端的三个坐标和身体的位姿矩阵 3*6+16=34
        		static double ee0[34];
        		double ee[34];

                //落地缓冲时间
                if (count() <= a)
                {
                    ret = 1;
                    if (count() == 1)
                    {
                        model()->getOutputPos(ee0); //初始位置
                        //s_vc好像是把ee0的数放到ee中，放34个数
                        aris::dynamic::s_vc(34, ee0, ee);
                    }
                    aris::dynamic::s_vc(34, ee0, ee);
                    model()->setOutputPos(ee);


                    if (model()->inverseKinematics()) std::cout << "inverse failed " << std::endl;
                    
                    model()->setTime(0.001 * count());
                }
                else
                {
                    TCurve s1(3, 1.5);
                    s1.getCurveParam();
                    EllipseTrajectory e1(0.1, 0.05, 0, s1);
                    BodyPose body_s(0, 0, 0, s1);


                    ret = tripodPlan(2, count() - 1-a, &e1, input_angle);
                    aris::dynamic::s_vc(16, file_current_body + 0, ee + 0);
                    aris::dynamic::s_vc(18, file_current_leg + 0, ee + 16);
                    ////末端位置
                    //for (int i = 0; i < 34; ++i)
                    //    lout() << ee[i] << "\t";
                    //lout() << std::endl;


                    //第一条腿电机的位置
                    //double leg1MotorPos[3] = { 0 };
                    //for (int i = 0; i < 3; ++i) {
                    //    lout() << input_angle[i] << "\t";
                    //}
                    //lout() << std::endl;

                    //解析解计算得到的输入的角度
                    //for (int i = 0; i < 18; ++i)
                    //    lout() << input_angle[i] << "\t";
                    //lout() << std::endl;

                    model()->setOutputPos(ee);
                    //model()->setInputPos(input_angle);
                    //if (model()->forwardKinematics()) {
                    //    std::cout << "Forward failer!" << std::endl;
                    //}


                    if (model()->inverseKinematics())
                    {
                        
                        std::cout << "inverse failed!!!" << std::endl;
                        //for (int i = 0; i < 34; ++i) {
                        //    std::cout << ee[i] << std::endl;
                        //}
                        std::cout << "ret = " << ret << std::endl;
                    }
                   // 数值解计算得到的输入的角度
                    //double input[18];
                    //model()->getInputPos(input);
                    //for (int i = 0; i < 18; ++i)
                    //    lout() << input[i] << "\t";
                    //lout() << std::endl;

                    model()->setTime(0.001 * count());


                    if (ret == 0) std::cout << count() << std::endl;

                }
                return ret;

        	}
            HexDynamicForwardTest::HexDynamicForwardTest(const std::string& name)
            {
                aris::core::fromXmlString(command(),
                    "<Command name=\"hex_forward\"/>");
            }
            HexDynamicForwardTest::~HexDynamicForwardTest() = default;




            //前进参数测试
            auto HexWalkingPrmTest::prepareNrt()->void
            {

            }
            auto HexWalkingPrmTest::executeRT()->int
            {
                //数值解和实际解xyr相差一个负号
                //如果要输出cmd文件，则不能创建储存文件，需要注释掉
                //if (count() == 1)this->master()->logFileRawName("eeTraj");    
                if (count() == 1)this->master()->logFileRawName("inputTraj");
                //if (count() == 1)this->master()->logFileRawName("invInput"); //反解计算结果储存文件，即解析解
                //if (count() == 1)this->master()->logFileRawName("numInput"); //数值解储存文件
                //if (count() == 1)this->master()->logFileRawName("leg1MotorPos1"); //拿到第1条腿的电机位置
                //if (count() == 1)this->master()->logFileRawName("leg1EndTraj1"); //拿到第1条腿的末端位置，要得到腿坐标系下的

                //a为给机器人缓冲落地的时间设置
                int ret = 0, a = 500;
                //末端为六个末端的三个坐标和身体的位姿矩阵 3*6+16=34
                static double ee0[34];
                double ee[34];


                //落地缓冲时间
                if (count() <= a)
                {
                   
                    if (count() == 1)
                    {
                        model()->getOutputPos(ee0); //初始位置
                        //s_vc好像是把ee0的数放到ee中，放34个数
                        aris::dynamic::s_vc(34, ee0, ee);
                    }


                    aris::dynamic::s_vc(34, ee0, ee);
                    model()->setOutputPos(ee);


                    if (model()->inverseKinematics()) std::cout << "inverse failed " << std::endl;

                    model()->setTime(0.001 * count());
                    ret = 1;
                }
                else if (a  < count() && count()<= (a + 1000))
                {
                    //std::cout << "count = " << count() << std::endl;
                    //函数1  根据输入腿1 坐标计算各腿函数末端
                    double all_ee[18] = { 0 };
                    double ee1[3] = { 0.4235,-0.4297,0 };
                    //double ee1[3] = { 0.68249-0.335,-0.37252,0 };
                    CalculateLegEE(ee1, all_ee); //all_ee中储存所有腿末端的位置

                    //函数2 输入所有腿末端坐标，生成直线轨迹到达指定末端


                    //coutMatrix18(foot_position_start_point);
                    TCurve s1(4, 2);
                    GenTrajToEE(all_ee, s1, count() - a);
                    aris::dynamic::s_vc(16, body_position_start_point + 0, ee);
                    aris::dynamic::s_vc(18, file_current_leg + 0, ee + 16);
                    model()->setOutputPos(ee);
                    if (model()->inverseKinematics())
                    {

                        std::cout << "inverse failed!!!" << std::endl;
                        //for (int i = 0; i < 34; ++i) {
                        //    std::cout << ee[i] << std::endl;
                        //}
                        std::cout << "ret = " << ret << std::endl;
                    }
                    model()->setTime(0.001 * count());
                    ret = 1;


                }
                else
                {

                    TCurve s1(3, 1.5);
                    s1.getCurveParam();
                    EllipseTrajectory e1(0.1, 0.03, 0, s1);
                    BodyPose body_s(0, 0, 0, s1);

                    if (count() == a  +1000+1)
                    {
                        aris::dynamic::s_vc(18, file_current_leg + 0, foot_position_start_point);
                    }
                    ret = tripodPlan(2, count() - 1 - a-1000, &e1, input_angle);
                    aris::dynamic::s_vc(16, file_current_body + 0, ee + 0);
                    aris::dynamic::s_vc(18, file_current_leg + 0, ee + 16);
                    ////末端位置
                    //for (int i = 0; i < 34; ++i)
                    //    lout() << ee[i] << "\t";
                    //lout() << std::endl;


                    //第一条腿电机的位置
                    //double leg1MotorPos[3] = { 0 };
                    //for (int i = 0; i < 3; ++i) {
                    //    lout() << input_angle[i] << "\t";
                    //}
                    //lout() << std::endl;

                    //解析解计算得到的输入的角度
                    //for (int i = 0; i < 18; ++i)
                    //    lout() << input_angle[i] << "\t";
                    //lout() << std::endl;

                    model()->setOutputPos(ee);
                    //model()->setInputPos(input_angle);
                    //if (model()->forwardKinematics()) {
                    //    std::cout << "Forward failer!" << std::endl;
                    //}


                    if (model()->inverseKinematics())
                    {

                        std::cout << "inverse failed!!!" << std::endl;
                        //for (int i = 0; i < 34; ++i) {
                        //    std::cout << ee[i] << std::endl;
                        //}
                        std::cout << "ret = " << ret << std::endl;
                    }
                    // 数值解计算得到的输入的角度
                     //double input[18];
                     //model()->getInputPos(input);
                     //for (int i = 0; i < 18; ++i)
                     //    lout() << input[i] << "\t";
                     //lout() << std::endl;

                    model()->setTime(0.001 * count());


                    if (ret == 0) std::cout << count() << std::endl;

                }
                //for (int i = 0; i < 34; ++i)
                //    lout() << ee[i] << "\t";
                //lout() << std::endl;
                double input[18];
                model()->getInputPos(input);
                for (int i = 0; i < 18; ++i)
                    lout() << input[i] << "\t";
                lout() << std::endl;
                
                return ret;

            }
            HexWalkingPrmTest::HexWalkingPrmTest(const std::string& name)
            {
                aris::core::fromXmlString(command(),
                    "<Command name=\"walk_prm\"/>");
            }
            HexWalkingPrmTest::~HexWalkingPrmTest() = default;


            //设置行走初始时刻的姿态，与实机用法略有不同，注意别搞混了
            //传入末端在腿坐标系下的坐标，leg1
            auto SetInitPos(double x, double y, double z)->void
            {
                double ee[3] = { x,y,z };

                foot_position_start_point[0] = ee[0] + FRONTX;
                foot_position_start_point[1] = ee[1];
                foot_position_start_point[2] = ee[2];
                CalculateInitPos();
                double mot[3] = { 0 };
                legInverseKinematics2(ee, mot, ans);
                legInverseKinematics(ee, mot);
            }

            //计算其他腿的初始位姿
            auto CalculateInitPos()->void
            {
                double ry60[16] = { cos(PI / 3),0,sin(PI / 3),0,
                                 0,1,0,0,
                                 -sin(PI / 3),0,cos(PI / 3),0,
                                 0,0,0,1 };

                aris::dynamic::s_pp2pp(ry60, foot_position_start_point + 0 * 3, foot_position_start_point + 1 * 3); //leg2
                aris::dynamic::s_pp2pp(ry60, foot_position_start_point + 1 * 3, foot_position_start_point + 2 * 3); //leg3
                aris::dynamic::s_pp2pp(ry60, foot_position_start_point + 2 * 3, foot_position_start_point + 3 * 3); //leg4
                aris::dynamic::s_pp2pp(ry60, foot_position_start_point + 3 * 3, foot_position_start_point + 4 * 3); //leg5
                aris::dynamic::s_pp2pp(ry60, foot_position_start_point + 4 * 3, foot_position_start_point + 5 * 3); //leg6
                std::copy(foot_position_start_point, foot_position_start_point + 18, foot_position_related_body);


            }

            //ee为腿1在腿坐标系下的坐标xyz
            auto CalculateLegEE(double *ee1,double* all_ee)->void
            {
                const double ry60[16] = { cos(PI / 3),0,sin(PI / 3),0,
                 0,1,0,0,
                 -sin(PI / 3),0,cos(PI / 3),0,
                 0,0,0,1 };

                ee1[0] += FRONTX;
                aris::dynamic::s_vc(3, ee1, all_ee); 
                aris::dynamic::s_pp2pp(ry60, all_ee + 0 * 3, all_ee + 1 * 3); //leg2
                aris::dynamic::s_pp2pp(ry60, all_ee + 1 * 3, all_ee + 2 * 3); //leg3
                aris::dynamic::s_pp2pp(ry60, all_ee + 2 * 3, all_ee + 3 * 3); //leg4
                aris::dynamic::s_pp2pp(ry60, all_ee + 3 * 3, all_ee + 4 * 3); //leg5
                aris::dynamic::s_pp2pp(ry60, all_ee + 4 * 3, all_ee + 5 * 3); //leg6
                //coutMatrix18(all_ee);
            }


            //产生腿移动到末端的轨迹
            auto GenTrajToEE(double* all_ee, TCurve& s1, int count)->void //count从零开始
            {
                //做差计算当前末端和指定末端相差的距离
                double ret = 0;
                double rel_distance[18] = { 0 };
                for (int i = 0; i < 18; i++) {
                    rel_distance[i] = all_ee[i] - foot_position_start_point[i];
                }
     
                s1.getCurveParam();
                StraightTrajectory leg_s0(rel_distance[0], rel_distance[1], rel_distance[2], s1);//此处传入三个坐标要移动的距离
                StraightTrajectory leg_s1(rel_distance[3], rel_distance[4], rel_distance[5], s1);
                StraightTrajectory leg_s2(rel_distance[6], rel_distance[7], rel_distance[8], s1);
                StraightTrajectory leg_s3(rel_distance[9], rel_distance[10], rel_distance[11], s1);
                StraightTrajectory leg_s4(rel_distance[12], rel_distance[13], rel_distance[14], s1);
                StraightTrajectory leg_s5(rel_distance[15], rel_distance[16], rel_distance[17], s1);

                ret = legStaightMovePlan(count, 0, &leg_s0);//一条腿对应一次函数调用，对应一个leg_s,leg_num 是从0->5
                ret = legStaightMovePlan(count, 1, &leg_s1);//一条腿对应一次函数调用，对应一个leg_s,leg_num 是从0->5
                ret = legStaightMovePlan(count, 2, &leg_s2);//一条腿对应一次函数调用，对应一个leg_s,leg_num 是从0->5
                ret = legStaightMovePlan(count, 3, &leg_s3);//一条腿对应一次函数调用，对应一个leg_s,leg_num 是从0->5
                ret = legStaightMovePlan(count, 4, &leg_s4);//一条腿对应一次函数调用，对应一个leg_s,leg_num 是从0->5
                ret = legStaightMovePlan(count, 5, &leg_s5);//一条腿对应一次函数调用，对应一个leg_s,leg_num 是从0->5

            }




            //侧移参数测试
            auto HexLateralPrmTest::prepareNrt()->void
            {

            }
            auto HexLateralPrmTest::executeRT()->int
            {
                //数值解和实际解xyr相差一个负号
                //如果要输出cmd文件，则不能创建储存文件，需要注释掉
                //if (count() == 1)this->master()->logFileRawName("eeTraj");    
                //if (count() == 1)this->master()->logFileRawName("inputTraj");
                //if (count() == 1)this->master()->logFileRawName("invInput"); //反解计算结果储存文件，即解析解
                //if (count() == 1)this->master()->logFileRawName("numInput"); //数值解储存文件
                //if (count() == 1)this->master()->logFileRawName("leg1MotorPos1"); //拿到第1条腿的电机位置
                //if (count() == 1)this->master()->logFileRawName("leg1EndTraj1"); //拿到第1条腿的末端位置，要得到腿坐标系下的

                //a为给机器人缓冲落地的时间设置
                int ret = 0, a = 500;
                //末端为六个末端的三个坐标和身体的位姿矩阵 3*6+16=34
                static double ee0[34];
                double ee[34];


                //落地缓冲时间
                if (count() <= a)
                {

                    if (count() == 1)
                    {
                        model()->getOutputPos(ee0); //初始位置
                        //s_vc好像是把ee0的数放到ee中，放34个数
                        aris::dynamic::s_vc(34, ee0, ee);
                    }


                    aris::dynamic::s_vc(34, ee0, ee);
                    model()->setOutputPos(ee);


                    if (model()->inverseKinematics()) std::cout << "inverse failed " << std::endl;

                    model()->setTime(0.001 * count());
                    ret = 1;
                }
                else if (a < count() && count() <= (a + 1000))
                {
                    //std::cout << "count = " << count() << std::endl;
                    //函数1  根据输入腿1 坐标计算各腿函数末端
                    double all_ee[18] = { 0 };
                    double ee1[3] = { 0.4325,-0.4427,0 };
                    //double ee1[3] = { 0.68249-0.335,-0.37252,0 };
                    CalculateLegEE(ee1, all_ee); //all_ee中储存所有腿末端的位置

                    //函数2 输入所有腿末端坐标，生成直线轨迹到达指定末端


                    //coutMatrix18(foot_position_start_point);
                    TCurve s1(4, 2);
                    GenTrajToEE(all_ee, s1, count() - a);
                    aris::dynamic::s_vc(16, body_position_start_point + 0, ee);
                    aris::dynamic::s_vc(18, file_current_leg + 0, ee + 16);
                    model()->setOutputPos(ee);
                    if (model()->inverseKinematics())
                    {

                        std::cout << "inverse failed!!!" << std::endl;
                        //for (int i = 0; i < 34; ++i) {
                        //    std::cout << ee[i] << std::endl;
                        //}
                        std::cout << "ret = " << ret << std::endl;
                    }
                    model()->setTime(0.001 * count());
                    ret = 1;


                }
                else
                {

                    TCurve s1(3, 1.5);
                    s1.getCurveParam();
                    EllipseTrajectory e1(0., 0.03, 0.1, s1);
                    BodyPose body_s(0, 0, 0, s1);

                    if (count() == a + 1000 + 1)
                    {
                        aris::dynamic::s_vc(18, file_current_leg + 0, foot_position_start_point);
                    }
                    ret = tripodPlan(2, count() - 1 - a - 1000, &e1, input_angle);
                    aris::dynamic::s_vc(16, file_current_body + 0, ee + 0);
                    aris::dynamic::s_vc(18, file_current_leg + 0, ee + 16);
                    ////末端位置
                    //for (int i = 0; i < 34; ++i)
                    //    lout() << ee[i] << "\t";
                    //lout() << std::endl;


                    //第一条腿电机的位置
                    //double leg1MotorPos[3] = { 0 };
                    //for (int i = 0; i < 3; ++i) {
                    //    lout() << input_angle[i] << "\t";
                    //}
                    //lout() << std::endl;

                    //解析解计算得到的输入的角度
                    //for (int i = 0; i < 18; ++i)
                    //    lout() << input_angle[i] << "\t";
                    //lout() << std::endl;

                    model()->setOutputPos(ee);
                    //model()->setInputPos(input_angle);
                    //if (model()->forwardKinematics()) {
                    //    std::cout << "Forward failer!" << std::endl;
                    //}


                    if (model()->inverseKinematics())
                    {

                        std::cout << "inverse failed!!!" << std::endl;
                        //for (int i = 0; i < 34; ++i) {
                        //    std::cout << ee[i] << std::endl;
                        //}
                        std::cout << "ret = " << ret << std::endl;
                    }
                    // 数值解计算得到的输入的角度
                     //double input[18];
                     //model()->getInputPos(input);
                     //for (int i = 0; i < 18; ++i)
                     //    lout() << input[i] << "\t";
                     //lout() << std::endl;

                    model()->setTime(0.001 * count());


                    if (ret == 0) std::cout << count() << std::endl;

                }
                //for (int i = 0; i < 34; ++i)
                //    lout() << ee[i] << "\t";
                //lout() << std::endl;

                return ret;

            }
            HexLateralPrmTest::HexLateralPrmTest(const std::string& name)
            {
                aris::core::fromXmlString(command(),
                    "<Command name=\"lateral_prm\"/>");
            }
            HexLateralPrmTest::~HexLateralPrmTest() = default;




            //右转向参数测试
            auto HexTurnPrmTest::prepareNrt()->void
            {

            }
            auto HexTurnPrmTest::executeRT()->int
            {
                //数值解和实际解xyr相差一个负号
                //如果要输出cmd文件，则不能创建储存文件，需要注释掉
                //if (count() == 1)this->master()->logFileRawName("eeTraj");    
                //if (count() == 1)this->master()->logFileRawName("inputTraj");
                //if (count() == 1)this->master()->logFileRawName("invInput"); //反解计算结果储存文件，即解析解
                //if (count() == 1)this->master()->logFileRawName("numInput"); //数值解储存文件
                //if (count() == 1)this->master()->logFileRawName("leg1MotorPos1"); //拿到第1条腿的电机位置
                //if (count() == 1)this->master()->logFileRawName("leg1EndTraj1"); //拿到第1条腿的末端位置，要得到腿坐标系下的

                //a为给机器人缓冲落地的时间设置
                int ret = 0, a = 500;
                //末端为六个末端的三个坐标和身体的位姿矩阵 3*6+16=34
                static double ee0[34];
                double ee[34];


                //落地缓冲时间
                if (count() <= a)
                {

                    if (count() == 1)
                    {
                        model()->getOutputPos(ee0); //初始位置
                        //s_vc好像是把ee0的数放到ee中，放34个数
                        aris::dynamic::s_vc(34, ee0, ee);
                    }


                    aris::dynamic::s_vc(34, ee0, ee);
                    model()->setOutputPos(ee);


                    if (model()->inverseKinematics()) std::cout << "inverse failed " << std::endl;

                    model()->setTime(0.001 * count());
                    ret = 1;
                }
                else if (a < count() && count() <= (a + 1000))
                {
                    //std::cout << "count = " << count() << std::endl;
                    //函数1  根据输入腿1 坐标计算各腿函数末端
                    double all_ee[18] = { 0 };
                    double ee1[3] = { 0.4325,-0.4427,0 };
                    //double ee1[3] = { 0.68249-0.335,-0.37252,0 };
                    CalculateLegEE(ee1, all_ee); //all_ee中储存所有腿末端的位置

                    //函数2 输入所有腿末端坐标，生成直线轨迹到达指定末端


                    //coutMatrix18(foot_position_start_point);
                    TCurve s1(4, 2);
                    GenTrajToEE(all_ee, s1, count() - a);
                    aris::dynamic::s_vc(16, body_position_start_point + 0, ee);
                    aris::dynamic::s_vc(18, file_current_leg + 0, ee + 16);
                    model()->setOutputPos(ee);
                    if (model()->inverseKinematics())
                    {

                        std::cout << "inverse failed!!!" << std::endl;
                        //for (int i = 0; i < 34; ++i) {
                        //    std::cout << ee[i] << std::endl;
                        //}
                        std::cout << "ret = " << ret << std::endl;
                    }
                    model()->setTime(0.001 * count());
                    ret = 1;


                }
                else
                {


                    TCurve s1(3, 1.5);
                    s1.getCurveParam();
                    EllipseTrajectory e1(0, 0.05, 0, s1);
                    BodyPose body_s(0, 15, 0, s1); //仿真时候看下动画方向是否一致（俯视图为逆时针，如果不是需要调整为逆时针）

                    if (count() == a + 1000 + 1)
                    {
                        aris::dynamic::s_vc(18, file_current_leg + 0, foot_position_start_point);
                    }


                    ret = turnPlanTripod(2, count() - 1 - a-1000, &e1, &body_s, input_angle);
                    aris::dynamic::s_vc(16, file_current_body + 0, ee + 0);
                    aris::dynamic::s_vc(18, file_current_leg + 0, ee + 16);


                    model()->setOutputPos(ee);

                    //末端位置
                    //for (int i = 0; i < 34; ++i)
                    //    lout() << ee[i] << "\t";
                    //lout() << std::endl;






                    if (model()->inverseKinematics())
                    {

                        std::cout << "inverse failed!!!" << std::endl;
                        //for (int i = 0; i < 34; ++i) {
                        //    std::cout << ee[i] << std::endl;
                        //}
                        std::cout << "ret = " << ret << std::endl;
                    }

                    // 数值解计算得到的输入的角度
                    //double input[18];
                    //model()->getInputPos(input);
                    //for (int i = 0; i < 18; ++i)
                    //    lout() << input[i] << "\t";
                    //lout() << std::endl;

                    model()->setTime(0.001 * count());


                    if (ret == 0) std::cout << count() << std::endl;




                }
                //for (int i = 0; i < 34; ++i)
                //    lout() << ee[i] << "\t";
                //lout() << std::endl;

                return ret;

            }
            HexTurnPrmTest::HexTurnPrmTest(const std::string& name)
            {
                aris::core::fromXmlString(command(),
                    "<Command name=\"turn_prm\"/>");
            }
            HexTurnPrmTest::~HexTurnPrmTest() = default;






            //后退
            auto HexDynamicBackTest::prepareNrt()->void
            {

            }
            auto HexDynamicBackTest::executeRT()->int
            {
                //如果要输出cmd文件，则不能创建储存文件，需要注释掉
                //if (count() == 1)this->master()->logFileRawName("eeTraj");    

                //a为给机器人缓冲落地的时间设置
                int ret = 0, a = 500;
                //末端为六个末端的三个坐标和身体的位姿矩阵 3*6+16=34
                static double ee0[34];
                double ee[34];

                //落地缓冲时间
                if (count() <= a)
                {
                    ret = 1;
                    if (count() == 1)
                    {
                        model()->getOutputPos(ee0); //初始位置
                        //s_vc好像是把ee0的数放到ee中，放34个数
                        aris::dynamic::s_vc(34, ee0, ee);
                    }
                    aris::dynamic::s_vc(34, ee0, ee);
                    model()->setOutputPos(ee);


                    if (model()->inverseKinematics()) std::cout << "inverse failed " << std::endl;

                    model()->setTime(0.001 * count());
                }
                else
                {
                    TCurve s1(4, 2);
                    s1.getCurveParam();
                    EllipseTrajectory e1(-0.1, 0.05, 0, s1);
                    BodyPose body_s(0, 0, 0, s1);


                    ret = tripodPlan(2, count() - 1 - a, &e1, input_angle);
                    aris::dynamic::s_vc(16, file_current_body + 0, ee + 0);
                    aris::dynamic::s_vc(18, file_current_leg + 0, ee + 16);


                    model()->setOutputPos(ee);



                    if (model()->inverseKinematics())
                    {

                        std::cout << "inverse failed!!!" << std::endl;
                        //for (int i = 0; i < 34; ++i) {
                        //    std::cout << ee[i] << std::endl;
                        //}
                        std::cout << "ret = " << ret << std::endl;
                    }

                    model()->setTime(0.001 * count());


                    if (ret == 0) std::cout << count() << std::endl;

                }
                return ret;
            }
            HexDynamicBackTest::HexDynamicBackTest(const std::string& name)
            {
                aris::core::fromXmlString(command(),
                    "<Command name=\"hex_back\"/>");
            }
            HexDynamicBackTest::~HexDynamicBackTest() = default;

            //右移
            auto HexDynamicRightTest::prepareNrt()->void
            {

            }
            auto HexDynamicRightTest::executeRT()->int
            {
                //如果要输出cmd文件，则不能创建储存文件，需要注释掉
                //if (count() == 1)this->master()->logFileRawName("eeTraj");    
                //if (count() == 1)this->master()->logFileRawName("leg1MotorPos"); //拿到第1条腿的电机位置
                if (count() == 1)this->master()->logFileRawName("leg1EndTraj"); //拿到第1条腿的末端位置，要得到腿坐标系下的

                //a为给机器人缓冲落地的时间设置
                int ret = 0, a = 500;
                //末端为六个末端的三个坐标和身体的位姿矩阵 3*6+16=34
                static double ee0[34];
                double ee[34];

                //落地缓冲时间
                if (count() <= a)
                {
                    ret = 1;
                    if (count() == 1)
                    {
                        model()->getOutputPos(ee0); //初始位置
                        //s_vc好像是把ee0的数放到ee中，放34个数
                        aris::dynamic::s_vc(34, ee0, ee);
                    }
                    aris::dynamic::s_vc(34, ee0, ee);
                    model()->setOutputPos(ee);


                    if (model()->inverseKinematics()) std::cout << "inverse failed " << std::endl;

                    model()->setTime(0.001 * count());
                }
                else
                {
                    TCurve s1(4, 2);
                    s1.getCurveParam();
                    EllipseTrajectory e1(0, 0.05, 0.1, s1);
                    BodyPose body_s(0, 0, 0, s1);


                    ret = tripodPlan(2, count() - 1 - a, &e1, input_angle);
                    aris::dynamic::s_vc(16, file_current_body + 0, ee + 0);
                    aris::dynamic::s_vc(18, file_current_leg + 0, ee + 16);


                    model()->setOutputPos(ee);


                    //for (int i = 0; i < 3; ++i) {
                    //    lout() << input_angle[i] << "\t";
                    //}
                    //lout() << std::endl;


                    //末端位置
                    for (int i = 0; i < 34; ++i)
                        lout() << ee[i] << "\t";
                    lout() << std::endl;



                    if (model()->inverseKinematics())
                    {

                        std::cout << "inverse failed!!!" << std::endl;
                        //for (int i = 0; i < 34; ++i) {
                        //    std::cout << ee[i] << std::endl;
                        //}
                        std::cout << "ret = " << ret << std::endl;
                    }

                    model()->setTime(0.001 * count());


                    if (ret == 0) std::cout << count() << std::endl;

                }
                return ret;
               
            }
            HexDynamicRightTest::HexDynamicRightTest(const std::string& name)
            {
                aris::core::fromXmlString(command(),
                    "<Command name=\"hex_right\"/>");
            }
            HexDynamicRightTest::~HexDynamicRightTest() = default;



            //左移
            auto HexDynamicLeftTest::prepareNrt()->void
            {

            }
            auto HexDynamicLeftTest::executeRT()->int
            {
                //如果要输出cmd文件，则不能创建储存文件，需要注释掉
               //if (count() == 1)this->master()->logFileRawName("eeTraj");    

               //a为给机器人缓冲落地的时间设置
                int ret = 0, a = 500;
                //末端为六个末端的三个坐标和身体的位姿矩阵 3*6+16=34
                static double ee0[34];
                double ee[34];

                //落地缓冲时间
                if (count() <= a)
                {
                    ret = 1;
                    if (count() == 1)
                    {
                        model()->getOutputPos(ee0); //初始位置
                        //s_vc好像是把ee0的数放到ee中，放34个数
                        aris::dynamic::s_vc(34, ee0, ee);
                    }
                    aris::dynamic::s_vc(34, ee0, ee);
                    model()->setOutputPos(ee);


                    if (model()->inverseKinematics()) std::cout << "inverse failed " << std::endl;

                    model()->setTime(0.001 * count());
                }
                else
                {
                    TCurve s1(4, 2);
                    s1.getCurveParam();
                    EllipseTrajectory e1(0, 0.05, -0.1, s1);
                    BodyPose body_s(0, 0, 0, s1);


                    ret = tripodPlan(2, count() - 1 - a, &e1, input_angle);
                    aris::dynamic::s_vc(16, file_current_body + 0, ee + 0);
                    aris::dynamic::s_vc(18, file_current_leg + 0, ee + 16);


                    model()->setOutputPos(ee);



                    if (model()->inverseKinematics())
                    {

                        std::cout << "inverse failed!!!" << std::endl;
                        //for (int i = 0; i < 34; ++i) {
                        //    std::cout << ee[i] << std::endl;
                        //}
                        std::cout << "ret = " << ret << std::endl;
                    }

                    model()->setTime(0.001 * count());


                    if (ret == 0) std::cout << count() << std::endl;

                }
                return ret;
            }
            HexDynamicLeftTest::HexDynamicLeftTest(const std::string& name)
            {
                aris::core::fromXmlString(command(),
                    "<Command name=\"hex_left\"/>");
            }
            HexDynamicLeftTest::~HexDynamicLeftTest() = default;


            //右转
            auto HexDynamicTurnRightTest::prepareNrt()->void
            {

            }
            auto HexDynamicTurnRightTest::executeRT()->int
            {

                //如果要输出cmd文件，则不能创建储存文件，需要注释掉
                //if (count() == 1)this->master()->logFileRawName("motInput");  
                if (count() == 1)this->master()->logFileRawName("leg1MotorPos2"); //拿到第1条腿的电机位置
                //if (count() == 1)this->master()->logFileRawName("leg1EndTraj2"); //拿到第1条腿的末端位置，要得到腿坐标系下的

                //a为给机器人缓冲落地的时间设置
                int ret = 0, a = 500;
                //末端为六个末端的三个坐标和身体的位姿矩阵 3*6+16=34
                static double ee0[34];
                double ee[34];

                //落地缓冲时间
                if (count() <= a)
                {
                    ret = 1;
                    if (count() == 1)
                    {
                        model()->getOutputPos(ee0); //初始位置
                        //s_vc好像是把ee0的数放到ee中，放34个数
                        aris::dynamic::s_vc(34, ee0, ee);
                    }
                    aris::dynamic::s_vc(34, ee0, ee);
                    model()->setOutputPos(ee);


                    if (model()->inverseKinematics()) std::cout << "inverse failed " << std::endl;

                    model()->setTime(0.001 * count());
                }
                else
                {
                    TCurve s1(4, 2);
                    s1.getCurveParam();
                    EllipseTrajectory e1(0, 0.05, 0, s1);
                    BodyPose body_s(0, -20, 0, s1); //目前看来就是这个正负号影响步长的改变


                    ret = turnPlanTripod(2, count() - 1 - a, &e1, &body_s, input_angle);
                    aris::dynamic::s_vc(16, file_current_body + 0, ee + 0);
                    aris::dynamic::s_vc(18, file_current_leg + 0, ee + 16);


                    model()->setOutputPos(ee);

                    //末端位置
                    //for (int i = 0; i < 34; ++i)
                    //    lout() << ee[i] << "\t";
                    //lout() << std::endl;


                    for (int i = 0; i < 3; ++i) {
                        lout() << input_angle[i] << "\t";
                    }
                    lout() << std::endl;



                    if (model()->inverseKinematics())
                    {

                        std::cout << "inverse failed!!!" << std::endl;
                        //for (int i = 0; i < 34; ++i) {
                        //    std::cout << ee[i] << std::endl;
                        //}
                        std::cout << "ret = " << ret << std::endl;
                    }

                    // 数值解计算得到的输入的角度
                    //double input[18];
                    //model()->getInputPos(input);
                    //for (int i = 0; i < 18; ++i)
                    //    lout() << input[i] << "\t";
                    //lout() << std::endl;

                    model()->setTime(0.001 * count());


                    if (ret == 0) std::cout << count() << std::endl;

                }
                return ret;

            }
            HexDynamicTurnRightTest::HexDynamicTurnRightTest(const std::string& name)
            {
                aris::core::fromXmlString(command(),
                    "<Command name=\"hex_turn_right\"/>");
            }
            HexDynamicTurnRightTest::~HexDynamicTurnRightTest() = default;

            //左转
            auto HexDynamicTurnLeftTest::prepareNrt()->void
            {

            }
            auto HexDynamicTurnLeftTest::executeRT()->int
            {
                //如果要输出cmd文件，则不能创建储存文件，需要注释掉
                //if (count() == 1)this->master()->logFileRawName("eeTraj");    

                //a为给机器人缓冲落地的时间设置
                int ret = 0, a = 500;
                //末端为六个末端的三个坐标和身体的位姿矩阵 3*6+16=34
                static double ee0[34];
                double ee[34];

                //落地缓冲时间
                if (count() <= a)
                {
                    ret = 1;
                    if (count() == 1)
                    {
                        model()->getOutputPos(ee0); //初始位置
                        //s_vc好像是把ee0的数放到ee中，放34个数
                        aris::dynamic::s_vc(34, ee0, ee);
                    }
                    aris::dynamic::s_vc(34, ee0, ee);
                    model()->setOutputPos(ee);


                    if (model()->inverseKinematics()) std::cout << "inverse failed " << std::endl;

                    model()->setTime(0.001 * count());
                }
                else
                {
                    TCurve s1(4, 2);
                    s1.getCurveParam();
                    EllipseTrajectory e1(0, 0.05, 0, s1);
                    BodyPose body_s(0, 20, 0, s1);


                    ret = turnPlanTripod(2, count() - 1 - a, &e1, &body_s, input_angle);
                    aris::dynamic::s_vc(16, file_current_body + 0, ee + 0);
                    aris::dynamic::s_vc(18, file_current_leg + 0, ee + 16);


                    model()->setOutputPos(ee);



                    if (model()->inverseKinematics())
                    {

                        std::cout << "inverse failed!!!" << std::endl;
                        //for (int i = 0; i < 34; ++i) {
                        //    std::cout << ee[i] << std::endl;
                        //}
                        std::cout << "ret = " << ret << std::endl;
                    }

                    model()->setTime(0.001 * count());


                    if (ret == 0) std::cout << count() << std::endl;

                }
                return ret;

            }
            HexDynamicTurnLeftTest::HexDynamicTurnLeftTest(const std::string& name)
            {
                aris::core::fromXmlString(command(),
                    "<Command name=\"hex_turn_right\"/>");
            }
            HexDynamicTurnLeftTest::~HexDynamicTurnLeftTest() = default;

            //四足步态
            auto HexDynamicTetrapodTest::prepareNrt()->void
            {

            }
            auto HexDynamicTetrapodTest::executeRT()->int
            {

                //如果要输出cmd文件，则不能创建储存文件，需要注释掉
                if (count() == 1)this->master()->logFileRawName("eeTraj");    

                //a为给机器人缓冲落地的时间设置
                int ret = 0, a = 500;
                //末端为六个末端的三个坐标和身体的位姿矩阵 3*6+16=34
                static double ee0[34];
                double ee[34];

                //落地缓冲时间
                if (count() <= a)
                {
                    ret = 1;
                    if (count() == 1)
                    {
                        model()->getOutputPos(ee0); //初始位置
                        //s_vc好像是把ee0的数放到ee中，放34个数
                        aris::dynamic::s_vc(34, ee0, ee);
                    }
                    aris::dynamic::s_vc(34, ee0, ee);
                    model()->setOutputPos(ee);


                    if (model()->inverseKinematics()) std::cout << "inverse failed " << std::endl;

                    model()->setTime(0.001 * count());
                }
                else
                {
                    TCurve s1(4, 2);
                    s1.getCurveParam();
                    EllipseTrajectory e1(0.1, 0.05, 0, s1);
                    BodyPose body_s(0, 0, 0, s1);


                    ret = tetrapodPlan(3, count() - 1 - a, &e1, input_angle);
                    aris::dynamic::s_vc(16, file_current_body + 0, ee + 0);
                    aris::dynamic::s_vc(18, file_current_leg + 0, ee + 16);


                    model()->setOutputPos(ee);

                    ////末端位置
                    for (int i = 0; i < 34; ++i)
                        lout() << ee[i] << "\t";
                    lout() << std::endl;

                    if (model()->inverseKinematics())
                    {

                        std::cout << "inverse failed!!!" << std::endl;
                        //for (int i = 0; i < 34; ++i) {
                        //    std::cout << ee[i] << std::endl;
                        //}
                        std::cout << "ret = " << ret << std::endl;
                    }

                    model()->setTime(0.001 * count());


                    if (ret == 0) std::cout << count() << std::endl;

                }
                return ret;

            }
            HexDynamicTetrapodTest::HexDynamicTetrapodTest(const std::string& name)
            {
                aris::core::fromXmlString(command(),
                    "<Command name=\"hex_tetrapod\"/>");
            }
            HexDynamicTetrapodTest::~HexDynamicTetrapodTest() = default;






auto createModelHexapod()->std::unique_ptr<aris::dynamic::Model>
            {
                std::unique_ptr<aris::dynamic::Model> hex = std::make_unique<aris::dynamic::Model>();
                // set gravity //
                const double gravity[6]{ 0.0,-9.8,0.0,0.0,0.0,0.0 };
                hex->environment().setGravity(gravity);

                //define joint pos //
                //此处都是在初始位置下测量,坐标系朝向与Adams一致
                const double leg1_pe[10][3]{
                    {FRONTX,    0,          0}, //转轴与底板底面交点， 即p点，第一个     转动副
                    {0.403,     0.06094,    0}, //y方向推杆，y方向为同步轮底面到原点距离 移动副  1
                    {0.403,     0.02136,    0}, //推杆与曲柄连接的交点    B点   2
                    {0.383,     -0.032,     0}, //A点         3
                    {0.51571,   0.0969,     0}, //C点  4
                    {0.54959,   0,          0}, //D点  5
                    {0.45992,   -0.08428,   0}, //F点  6
                    {0.41688,   -0.12609,   0}, //G点 注意G点应该有两个转动副   7
                    {0.39214,   -0.12250,   0}, //H点   8
                    {0.321,     -0.1225,    0}, //X向推杆，距离为腿壳的表面圆心到中心的距离 移动副   9

                };

                const double leg2_pe[10][3]{
                    {0.1675,    0,          -0.2901},
                    {0.2015,    0.0609,     -0.3490},
                    {0.2015,    0.0214,     -0.3490},
                    {0.1915,    -0.0320,    -0.3317},
                    {0.2579,    0.0969,     -0.4466},
                    {0.2748,    0,          -0.4760},
                    {0.2300,    -0.0843,    -0.3983},
                    {0.2084,    -0.1261,    -0.3610},
                    {0.1961,    -0.1225,    -0.3396},
                    {0.1605,    -0.1225,    -0.2780}
                };

                const double leg3_pe[10][3]{
                    {-0.1675,    0,          -0.2901},
                    {-0.2015,    0.0609,     -0.3490},
                    {-0.2015,    0.0214,     -0.3490},
                    {-0.1915,    -0.0320,    -0.3317},
                    {-0.2579,    0.0969,     -0.4466},
                    {-0.2748,    0,          -0.4760},
                    {-0.2300,    -0.0843,    -0.3983},
                    {-0.2084,    -0.1261,    -0.3610},
                    {-0.1961,    -0.1225,    -0.3396},
                    {-0.1605,    -0.1225,    -0.2780}
                };

                const double leg4_pe[10][3]{
                    {-FRONTX,    0,          0}, //转轴与底板底面交点， 即p点，第一个     转动副
                    {-0.403,     0.06094,    0}, //y方向推杆，y方向为同步轮底面到原点距离 移动副  1
                    {-0.403,     0.02136,    0}, //推杆与曲柄连接的交点    B点   2
                    {-0.383,     -0.032,     0}, //A点         3
                    {-0.51571,   0.0969,     0}, //C点  4
                    {-0.54959,   0,          0}, //D点  5
                    {-0.45992,   -0.08428,   0}, //F点  6
                    {-0.41688,   -0.12609,   0}, //G点 注意G点应该有两个转动副   7
                    {-0.39214,   -0.12250,   0}, //H点   8
                    {-0.321,     -0.1225,    0}, //X向推杆，距离为腿壳的表面圆心到中心的距离 移动副   9

                };

                const double leg5_pe[10][3]{
                    {-0.1675,    0,          0.2901},
                    {-0.2015,    0.0609,     0.3490},
                    {-0.2015,    0.0214,     0.3490},
                    {-0.1915,    -0.0320,    0.3317},
                    {-0.2579,    0.0969,     0.4466},
                    {-0.2748,    0,          0.4760},
                    {-0.2300,    -0.0843,    0.3983},
                    {-0.2084,    -0.1261,    0.3610},
                    {-0.1961,    -0.1225,    0.3396},
                    {-0.1605,    -0.1225,    0.2780}
                };

                const double leg6_pe[10][3]{
                    {0.1675,    0,          0.2901},
                    {0.2015,    0.0609,     0.3490},
                    {0.2015,    0.0214,     0.3490},
                    {0.1915,    -0.0320,    0.3317},
                    {0.2579,    0.0969,     0.4466},
                    {0.2748,    0,          0.4760},
                    {0.2300,    -0.0843,    0.3983},
                    {0.2084,    -0.1261,    0.3610},
                    {0.1961,    -0.1225,    0.3396},
                    {0.1605,    -0.1225,    0.2780}
                };

                //define ee pos  六条腿的末端//
                const double ee_pos[6][6]
                {
                    {EE1_X,         -HEIGHT,        0.0,		    0.0,    0.0,    0.0},
                    {EE6_X,	        -HEIGHT,        -EE6_Z,		    0.0,    0.0,    0.0},
                    {-EE6_X,        -HEIGHT,        -EE6_Z,		    0.0,    0.0,    0.0},
                    {-EE1_X,        -HEIGHT,        0,			    0.0,    0.0,    0.0},
                    {-EE6_X,        -HEIGHT,        EE6_Z,		    0.0,    0.0,    0.0},
                    {EE6_X,	        -HEIGHT,        EE6_Z,		    0.0,    0.0,    0.0},
                };


                //这部分物理参数在最新的模型中还暂未修改
                //iv:  10x1 惯量矩阵向量[m, cx, cy, cz, Ixx, Iyy, Izz, Ixy, Ixz, Iyz]
                const double body_iv[10]{ 22.99,0,0,0,0.792149,0.792441,1.201794,0.000323,0.000351,0.000161 };
                //每条腿都在自己坐标系下，故惯性张量一样，不作区分，单腿重4.208793kg  总重35.872758kg，还有部分组件没有考虑上，估计在50kg左右
                const double leg_shell_iv[10]{5.185207,0,0,0,0.067462,0.016493,0.080198,0.013121,0.000005,0.000012};
                const double y_screw_iv[10]{0.138307,0,0,0,0.000286,0.000021,0.000302,0.000025,0.00,0.00};
                const double top_bar_iv[10]{0.206814,0,0,0,0.000519,0.000433,0.000674,0.000315,0.0,0.000001};
                const double longest_bar_iv[10]{0.566247,0,0,0,0.012401,0.000164,0.012383,0.000681,0.000000,0.000001};
                const double bot_bar_iv[10]{0.123833,0,0,0,0.000292,0.000226,0.000495,0.000241,0.0,0.0};
                const double crank_iv[10]{0.117524,0,0,0,0.000175,0.000033,0.000197,0.000043,0.0,0.0};
                const double h_bar_iv[10]{0.118757,0,0,0,0.000123,0.000042,0.000094,0.000004,0.0,0.0};
                const double shortest_bar_iv[10]{0.030776,0,0,0,0.000007,0.000009,0.000007,0,0,0};
                const double x_screw_iv[10]{0.101926,0,0,0,0.000006,0.000232,0.000234,0.000006,0.0,0.0};

                //add part //
                auto& body = hex->partPool().add<aris::dynamic::Part>("BODY", body_iv);
                //leg1
                auto& leg1_leg_shell = hex->partPool().add<aris::dynamic::Part>("leg1_leg_shell", leg_shell_iv);
                auto& leg1_y_screw = hex->partPool().add<aris::dynamic::Part>("leg1_y_screw", y_screw_iv);
                auto& leg1_top_bar = hex->partPool().add<aris::dynamic::Part>("leg1_top_bar", top_bar_iv);
                auto& leg1_longest_bar = hex->partPool().add<aris::dynamic::Part>("leg1_longest_bar", longest_bar_iv);
                auto& leg1_bot_bar = hex->partPool().add<aris::dynamic::Part>("leg1_bot_bar", bot_bar_iv);
                auto& leg1_crank = hex->partPool().add<aris::dynamic::Part>("leg1_crank", crank_iv);
                auto& leg1_h_bar = hex->partPool().add<aris::dynamic::Part>("leg1_h_bar", h_bar_iv);
                auto& leg1_shortest_bar = hex->partPool().add<aris::dynamic::Part>("leg1_shortest_bar", shortest_bar_iv);
                auto& leg1_x_screw = hex->partPool().add<aris::dynamic::Part>("leg1_x_screw", x_screw_iv);
                //leg2
                auto& leg2_leg_shell = hex->partPool().add<aris::dynamic::Part>("leg2_leg_shell", leg_shell_iv);
                auto& leg2_y_screw = hex->partPool().add<aris::dynamic::Part>("leg2_y_screw", y_screw_iv);
                auto& leg2_top_bar = hex->partPool().add<aris::dynamic::Part>("leg2_top_bar", top_bar_iv);
                auto& leg2_longest_bar = hex->partPool().add<aris::dynamic::Part>("leg2_longest_bar", longest_bar_iv);
                auto& leg2_bot_bar = hex->partPool().add<aris::dynamic::Part>("leg2_bot_bar", bot_bar_iv);
                auto& leg2_crank = hex->partPool().add<aris::dynamic::Part>("leg2_crank", crank_iv);
                auto& leg2_h_bar = hex->partPool().add<aris::dynamic::Part>("leg2_h_bar", h_bar_iv);
                auto& leg2_shortest_bar = hex->partPool().add<aris::dynamic::Part>("leg2_shortest_bar", shortest_bar_iv);
                auto& leg2_x_screw = hex->partPool().add<aris::dynamic::Part>("leg2_x_screw", x_screw_iv);
                //leg3
                auto& leg3_leg_shell = hex->partPool().add<aris::dynamic::Part>("leg3_leg_shell", leg_shell_iv);
                auto& leg3_y_screw = hex->partPool().add<aris::dynamic::Part>("leg3_y_screw", y_screw_iv);
                auto& leg3_top_bar = hex->partPool().add<aris::dynamic::Part>("leg3_top_bar", top_bar_iv);
                auto& leg3_longest_bar = hex->partPool().add<aris::dynamic::Part>("leg3_longest_bar", longest_bar_iv);
                auto& leg3_bot_bar = hex->partPool().add<aris::dynamic::Part>("leg3_bot_bar", bot_bar_iv);
                auto& leg3_crank = hex->partPool().add<aris::dynamic::Part>("leg3_crank", crank_iv);
                auto& leg3_h_bar = hex->partPool().add<aris::dynamic::Part>("leg3_h_bar", h_bar_iv);
                auto& leg3_shortest_bar = hex->partPool().add<aris::dynamic::Part>("leg3_shortest_bar", shortest_bar_iv);
                auto& leg3_x_screw = hex->partPool().add<aris::dynamic::Part>("leg3_x_screw", x_screw_iv);
                //leg4
                auto& leg4_leg_shell = hex->partPool().add<aris::dynamic::Part>("leg4_leg_shell", leg_shell_iv);
                auto& leg4_y_screw = hex->partPool().add<aris::dynamic::Part>("leg4_y_screw", y_screw_iv);
                auto& leg4_top_bar = hex->partPool().add<aris::dynamic::Part>("leg4_top_bar", top_bar_iv);
                auto& leg4_longest_bar = hex->partPool().add<aris::dynamic::Part>("leg4_longest_bar", longest_bar_iv);
                auto& leg4_bot_bar = hex->partPool().add<aris::dynamic::Part>("leg4_bot_bar", bot_bar_iv);
                auto& leg4_crank = hex->partPool().add<aris::dynamic::Part>("leg4_crank", crank_iv);
                auto& leg4_h_bar = hex->partPool().add<aris::dynamic::Part>("leg4_h_bar", h_bar_iv);
                auto& leg4_shortest_bar = hex->partPool().add<aris::dynamic::Part>("leg4_shortest_bar", shortest_bar_iv);
                auto& leg4_x_screw = hex->partPool().add<aris::dynamic::Part>("leg4_x_screw", x_screw_iv);
                //leg5
                auto& leg5_leg_shell = hex->partPool().add<aris::dynamic::Part>("leg5_leg_shell", leg_shell_iv);
                auto& leg5_y_screw = hex->partPool().add<aris::dynamic::Part>("leg5_y_screw", y_screw_iv);
                auto& leg5_top_bar = hex->partPool().add<aris::dynamic::Part>("leg5_top_bar", top_bar_iv);
                auto& leg5_longest_bar = hex->partPool().add<aris::dynamic::Part>("leg5_longest_bar", longest_bar_iv);
                auto& leg5_bot_bar = hex->partPool().add<aris::dynamic::Part>("leg5_bot_bar", bot_bar_iv);
                auto& leg5_crank = hex->partPool().add<aris::dynamic::Part>("leg5_crank", crank_iv);
                auto& leg5_h_bar = hex->partPool().add<aris::dynamic::Part>("leg5_h_bar", h_bar_iv);
                auto& leg5_shortest_bar = hex->partPool().add<aris::dynamic::Part>("leg5_shortest_bar", shortest_bar_iv);
                auto& leg5_x_screw = hex->partPool().add<aris::dynamic::Part>("leg5_x_screw", x_screw_iv);
                //leg6
                auto& leg6_leg_shell = hex->partPool().add<aris::dynamic::Part>("leg6_leg_shell", leg_shell_iv);
                auto& leg6_y_screw = hex->partPool().add<aris::dynamic::Part>("leg6_y_screw", y_screw_iv);
                auto& leg6_top_bar = hex->partPool().add<aris::dynamic::Part>("leg6_top_bar", top_bar_iv);
                auto& leg6_longest_bar = hex->partPool().add<aris::dynamic::Part>("leg6_longest_bar", longest_bar_iv);
                auto& leg6_bot_bar = hex->partPool().add<aris::dynamic::Part>("leg6_bot_bar", bot_bar_iv);
                auto& leg6_crank = hex->partPool().add<aris::dynamic::Part>("leg6_crank", crank_iv);
                auto& leg6_h_bar = hex->partPool().add<aris::dynamic::Part>("leg6_h_bar", h_bar_iv);
                auto& leg6_shortest_bar = hex->partPool().add<aris::dynamic::Part>("leg6_shortest_bar", shortest_bar_iv);
                auto& leg6_x_screw = hex->partPool().add<aris::dynamic::Part>("leg6_x_screw", x_screw_iv);



                //add geometry //
                hex->ground().geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\ground.x_t");
                body.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\BODY.x_t");
                // leg1
                leg1_leg_shell.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg1_leg_shell.x_t");
                leg1_y_screw.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg1_y_screw.x_t");
                leg1_top_bar.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg1_top_bar.x_t");
                leg1_longest_bar.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg1_longest_bar.x_t");
                leg1_bot_bar.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg1_bot_bar.x_t");
                leg1_crank.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg1_crank.x_t");
                leg1_h_bar.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg1_h_bar.x_t");
                leg1_shortest_bar.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg1_shortest_bar.x_t");
                leg1_x_screw.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg1_x_screw.x_t");
                // leg2
                leg2_leg_shell.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg2_leg_shell.x_t");
                leg2_y_screw.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg2_y_screw.x_t");
                leg2_top_bar.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg2_top_bar.x_t");
                leg2_longest_bar.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg2_longest_bar.x_t");
                leg2_bot_bar.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg2_bot_bar.x_t");
                leg2_crank.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg2_crank.x_t");
                leg2_h_bar.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg2_h_bar.x_t");
                leg2_shortest_bar.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg2_shortest_bar.x_t");
                leg2_x_screw.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg2_x_screw.x_t");
                // leg3
                leg3_leg_shell.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg3_leg_shell.x_t");
                leg3_y_screw.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg3_y_screw.x_t");
                leg3_top_bar.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg3_top_bar.x_t");
                leg3_longest_bar.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg3_longest_bar.x_t");
                leg3_bot_bar.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg3_bot_bar.x_t");
                leg3_crank.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg3_crank.x_t");
                leg3_h_bar.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg3_h_bar.x_t");
                leg3_shortest_bar.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg3_shortest_bar.x_t");
                leg3_x_screw.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg3_x_screw.x_t");
                // leg4
                leg4_leg_shell.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg4_leg_shell.x_t");
                leg4_y_screw.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg4_y_screw.x_t");
                leg4_top_bar.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg4_top_bar.x_t");
                leg4_longest_bar.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg4_longest_bar.x_t");
                leg4_bot_bar.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg4_bot_bar.x_t");
                leg4_crank.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg4_crank.x_t");
                leg4_h_bar.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg4_h_bar.x_t");
                leg4_shortest_bar.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg4_shortest_bar.x_t");
                leg4_x_screw.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg4_x_screw.x_t");
                // leg5
                leg5_leg_shell.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg5_leg_shell.x_t");
                leg5_y_screw.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg5_y_screw.x_t");
                leg5_top_bar.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg5_top_bar.x_t");
                leg5_longest_bar.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg5_longest_bar.x_t");
                leg5_bot_bar.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg5_bot_bar.x_t");
                leg5_crank.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg5_crank.x_t");
                leg5_h_bar.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg5_h_bar.x_t");
                leg5_shortest_bar.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg5_shortest_bar.x_t");
                leg5_x_screw.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg5_x_screw.x_t");
                // leg6
                leg6_leg_shell.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg6_leg_shell.x_t");
                leg6_y_screw.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg6_y_screw.x_t");
                leg6_top_bar.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg6_top_bar.x_t");
                leg6_longest_bar.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg6_longest_bar.x_t");
                leg6_bot_bar.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg6_bot_bar.x_t");
                leg6_crank.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg6_crank.x_t");
                leg6_h_bar.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg6_h_bar.x_t");
                leg6_shortest_bar.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg6_shortest_bar.x_t");
                leg6_x_screw.geometryPool().add<aris::dynamic::ParasolidGeometry>("C:\\Users\\jpche\\Desktop\\Adams_model2\\leg6_x_screw.x_t");

                //add joints//

                // leg1
                auto& leg1_r1 = hex->addRevoluteJoint(leg1_leg_shell, body, leg1_pe[0], std::array<double, 3>{0, 1, 0}.data());
                auto& leg1_p1 = hex->addPrismaticJoint(leg1_leg_shell, leg1_y_screw, leg1_pe[1], std::array<double, 3>{0, 1, 0}.data());
                auto& leg1_r2 = hex->addRevoluteJoint(leg1_y_screw, leg1_crank, leg1_pe[2], std::array<double, 3>{0, 0, 1}.data());
                auto& leg1_r3 = hex->addRevoluteJoint(leg1_leg_shell, leg1_top_bar, leg1_pe[3], std::array<double, 3>{0, 0, 1}.data());
                auto& leg1_r4 = hex->addRevoluteJoint(leg1_top_bar, leg1_h_bar, leg1_pe[3], std::array<double, 3>{0, 0, 1}.data());
                auto& leg1_r5 = hex->addRevoluteJoint(leg1_top_bar, leg1_longest_bar, leg1_pe[4], std::array<double, 3>{0, 0, 1}.data());
                auto& leg1_r6 = hex->addRevoluteJoint(leg1_longest_bar, leg1_bot_bar, leg1_pe[5], std::array<double, 3>{0, 0, 1}.data());
                auto& leg1_r7 = hex->addRevoluteJoint(leg1_bot_bar, leg1_crank, leg1_pe[6], std::array<double, 3>{0, 0, 1}.data());
                auto& leg1_r8 = hex->addRevoluteJoint(leg1_bot_bar, leg1_h_bar, leg1_pe[7], std::array<double, 3>{0, 0, 1}.data());
                auto& leg1_r9 = hex->addRevoluteJoint(leg1_bot_bar, leg1_shortest_bar, leg1_pe[7], std::array<double, 3>{0, 0, 1}.data());
                auto& leg1_r10 = hex->addRevoluteJoint(leg1_shortest_bar, leg1_x_screw, leg1_pe[8], std::array<double, 3>{0, 0, 1}.data());
                auto& leg1_p2 = hex->addPrismaticJoint(leg1_leg_shell, leg1_x_screw, leg1_pe[9], std::array<double, 3>{-1, 0, 0}.data());

                // leg2
                auto& leg2_r1 = hex->addRevoluteJoint(leg2_leg_shell, body, leg2_pe[0], std::array<double, 3>{0, 1, 0}.data());
                auto& leg2_p1 = hex->addPrismaticJoint(leg2_leg_shell, leg2_y_screw, leg2_pe[1], std::array<double, 3>{0, 1, 0}.data());
                auto& leg2_r2 = hex->addRevoluteJoint(leg2_y_screw, leg2_crank, leg2_pe[2], std::array<double, 3>{0.866, 0, 0.5}.data());
                auto& leg2_r3 = hex->addRevoluteJoint(leg2_leg_shell, leg2_top_bar, leg2_pe[3], std::array<double, 3>{0.866, 0, 0.5}.data());
                auto& leg2_r4 = hex->addRevoluteJoint(leg2_top_bar, leg2_h_bar, leg2_pe[3], std::array<double, 3>{0.866, 0, 0.5}.data());
                auto& leg2_r5 = hex->addRevoluteJoint(leg2_top_bar, leg2_longest_bar, leg2_pe[4], std::array<double, 3>{0.866, 0, 0.5}.data());
                auto& leg2_r6 = hex->addRevoluteJoint(leg2_longest_bar, leg2_bot_bar, leg2_pe[5], std::array<double, 3>{0.866, 0, 0.5}.data());
                auto& leg2_r7 = hex->addRevoluteJoint(leg2_bot_bar, leg2_crank, leg2_pe[6], std::array<double, 3>{0.866, 0, 0.5}.data());
                auto& leg2_r8 = hex->addRevoluteJoint(leg2_bot_bar, leg2_h_bar, leg2_pe[7], std::array<double, 3>{0.866, 0, 0.5}.data());
                auto& leg2_r9 = hex->addRevoluteJoint(leg2_bot_bar, leg2_shortest_bar, leg2_pe[7], std::array<double, 3>{0.866, 0, 0.5}.data());
                auto& leg2_r10 = hex->addRevoluteJoint(leg2_shortest_bar, leg2_x_screw, leg2_pe[8], std::array<double, 3>{0.866, 0, 0.5}.data());
                auto& leg2_p2 = hex->addPrismaticJoint(leg2_leg_shell, leg2_x_screw, leg2_pe[9], std::array<double, 3>{-0.5, 0, 0.866}.data());

                // leg3
                auto& leg3_r1 = hex->addRevoluteJoint(leg3_leg_shell, body, leg3_pe[0], std::array<double, 3>{0, 1, 0}.data());
                auto& leg3_p1 = hex->addPrismaticJoint(leg3_leg_shell, leg3_y_screw, leg3_pe[1], std::array<double, 3>{0, 1, 0}.data());
                auto& leg3_r2 = hex->addRevoluteJoint(leg3_y_screw, leg3_crank, leg3_pe[2], std::array<double, 3>{0.866, 0, -0.5}.data());
                auto& leg3_r3 = hex->addRevoluteJoint(leg3_leg_shell, leg3_top_bar, leg3_pe[3], std::array<double, 3>{0.866, 0, -0.5}.data());
                auto& leg3_r4 = hex->addRevoluteJoint(leg3_top_bar, leg3_h_bar, leg3_pe[3], std::array<double, 3>{0.866, 0, -0.5}.data());
                auto& leg3_r5 = hex->addRevoluteJoint(leg3_top_bar, leg3_longest_bar, leg3_pe[4], std::array<double, 3>{0.866, 0, -0.5}.data());
                auto& leg3_r6 = hex->addRevoluteJoint(leg3_longest_bar, leg3_bot_bar, leg3_pe[5], std::array<double, 3>{0.866, 0, -0.5}.data());
                auto& leg3_r7 = hex->addRevoluteJoint(leg3_bot_bar, leg3_crank, leg3_pe[6], std::array<double, 3>{0.866, 0, -0.5}.data());
                auto& leg3_r8 = hex->addRevoluteJoint(leg3_bot_bar, leg3_h_bar, leg3_pe[7], std::array<double, 3>{0.866, 0, -0.5}.data());
                auto& leg3_r9 = hex->addRevoluteJoint(leg3_bot_bar, leg3_shortest_bar, leg3_pe[7], std::array<double, 3>{0.866, 0, -0.5}.data());
                auto& leg3_r10 = hex->addRevoluteJoint(leg3_shortest_bar, leg3_x_screw, leg3_pe[8], std::array<double, 3>{0.866, 0, -0.5}.data());
                auto& leg3_p2 = hex->addPrismaticJoint(leg3_leg_shell, leg3_x_screw, leg3_pe[9], std::array<double, 3>{0.5, 0, 0.866}.data());

                // leg4
                auto& leg4_r1 = hex->addRevoluteJoint(leg4_leg_shell, body, leg4_pe[0], std::array<double, 3>{0, 1, 0}.data());
                auto& leg4_p1 = hex->addPrismaticJoint(leg4_leg_shell, leg4_y_screw, leg4_pe[1], std::array<double, 3>{0, 1, 0}.data());
                auto& leg4_r2 = hex->addRevoluteJoint(leg4_y_screw, leg4_crank, leg4_pe[2], std::array<double, 3>{0, 0, -1}.data());
                auto& leg4_r3 = hex->addRevoluteJoint(leg4_leg_shell, leg4_top_bar, leg4_pe[3], std::array<double, 3>{0, 0, -1}.data());
                auto& leg4_r4 = hex->addRevoluteJoint(leg4_top_bar, leg4_h_bar, leg4_pe[3], std::array<double, 3>{0, 0, -1}.data());
                auto& leg4_r5 = hex->addRevoluteJoint(leg4_top_bar, leg4_longest_bar, leg4_pe[4], std::array<double, 3>{0, 0, -1}.data());
                auto& leg4_r6 = hex->addRevoluteJoint(leg4_longest_bar, leg4_bot_bar, leg4_pe[5], std::array<double, 3>{0, 0, -1}.data());
                auto& leg4_r7 = hex->addRevoluteJoint(leg4_bot_bar, leg4_crank, leg4_pe[6], std::array<double, 3>{0, 0, -1}.data());
                auto& leg4_r8 = hex->addRevoluteJoint(leg4_bot_bar, leg4_h_bar, leg4_pe[7], std::array<double, 3>{0, 0, -1}.data());
                auto& leg4_r9 = hex->addRevoluteJoint(leg4_bot_bar, leg4_shortest_bar, leg4_pe[7], std::array<double, 3>{0, 0, -1}.data());
                auto& leg4_r10 = hex->addRevoluteJoint(leg4_shortest_bar, leg4_x_screw, leg4_pe[8], std::array<double, 3>{0, 0, -1}.data());
                auto& leg4_p2 = hex->addPrismaticJoint(leg4_leg_shell, leg4_x_screw, leg4_pe[9], std::array<double, 3>{1, 0, 0}.data());

                // leg5
                auto& leg5_r1 = hex->addRevoluteJoint(leg5_leg_shell, body, leg5_pe[0], std::array<double, 3>{0, 1, 0}.data());
                auto& leg5_p1 = hex->addPrismaticJoint(leg5_leg_shell, leg5_y_screw, leg5_pe[1], std::array<double, 3>{0, 1, 0}.data());
                auto& leg5_r2 = hex->addRevoluteJoint(leg5_y_screw, leg5_crank, leg5_pe[2], std::array<double, 3>{-0.866, 0, -0.5}.data());
                auto& leg5_r3 = hex->addRevoluteJoint(leg5_leg_shell, leg5_top_bar, leg5_pe[3], std::array<double, 3>{-0.866, 0, -0.5}.data());
                auto& leg5_r4 = hex->addRevoluteJoint(leg5_top_bar, leg5_h_bar, leg5_pe[3], std::array<double, 3>{-0.866, 0, -0.5}.data());
                auto& leg5_r5 = hex->addRevoluteJoint(leg5_top_bar, leg5_longest_bar, leg5_pe[4], std::array<double, 3>{-0.866, 0, -0.5}.data());
                auto& leg5_r6 = hex->addRevoluteJoint(leg5_longest_bar, leg5_bot_bar, leg5_pe[5], std::array<double, 3>{-0.866, 0, -0.5}.data());
                auto& leg5_r7 = hex->addRevoluteJoint(leg5_bot_bar, leg5_crank, leg5_pe[6], std::array<double, 3>{-0.866, 0, -0.5}.data());
                auto& leg5_r8 = hex->addRevoluteJoint(leg5_bot_bar, leg5_h_bar, leg5_pe[7], std::array<double, 3>{-0.866, 0, -0.5}.data());
                auto& leg5_r9 = hex->addRevoluteJoint(leg5_bot_bar, leg5_shortest_bar, leg5_pe[7], std::array<double, 3>{-0.866, 0, -0.5}.data());
                auto& leg5_r10 = hex->addRevoluteJoint(leg5_shortest_bar, leg5_x_screw, leg5_pe[8], std::array<double, 3>{-0.866, 0, -0.5}.data());
                auto& leg5_p2 = hex->addPrismaticJoint(leg5_leg_shell, leg5_x_screw, leg5_pe[9], std::array<double, 3>{0.5, 0, -0.866}.data());

                // leg6
                auto& leg6_r1 = hex->addRevoluteJoint(leg6_leg_shell, body, leg6_pe[0], std::array<double, 3>{0, 1, 0}.data());
                auto& leg6_p1 = hex->addPrismaticJoint(leg6_leg_shell, leg6_y_screw, leg6_pe[1], std::array<double, 3>{0, 1, 0}.data());
                auto& leg6_r2 = hex->addRevoluteJoint(leg6_y_screw, leg6_crank, leg6_pe[2], std::array<double, 3>{-0.866, 0, 0.5}.data());
                auto& leg6_r3 = hex->addRevoluteJoint(leg6_leg_shell, leg6_top_bar, leg6_pe[3], std::array<double, 3>{-0.866, 0, 0.5}.data());
                auto& leg6_r4 = hex->addRevoluteJoint(leg6_top_bar, leg6_h_bar, leg6_pe[3], std::array<double, 3>{-0.866, 0, 0.5}.data());
                auto& leg6_r5 = hex->addRevoluteJoint(leg6_top_bar, leg6_longest_bar, leg6_pe[4], std::array<double, 3>{-0.866, 0, 0.5}.data());
                auto& leg6_r6 = hex->addRevoluteJoint(leg6_longest_bar, leg6_bot_bar, leg6_pe[5], std::array<double, 3>{-0.866, 0, 0.5}.data());
                auto& leg6_r7 = hex->addRevoluteJoint(leg6_bot_bar, leg6_crank, leg6_pe[6], std::array<double, 3>{-0.866, 0, 0.5}.data());
                auto& leg6_r8 = hex->addRevoluteJoint(leg6_bot_bar, leg6_h_bar, leg6_pe[7], std::array<double, 3>{-0.866, 0, 0.5}.data());
                auto& leg6_r9 = hex->addRevoluteJoint(leg6_bot_bar, leg6_shortest_bar, leg6_pe[7], std::array<double, 3>{-0.866, 0, 0.5}.data());
                auto& leg6_r10 = hex->addRevoluteJoint(leg6_shortest_bar, leg6_x_screw, leg6_pe[8], std::array<double, 3>{-0.866, 0, 0.5}.data());
                auto& leg6_p2 = hex->addPrismaticJoint(leg6_leg_shell, leg6_x_screw, leg6_pe[9], std::array<double, 3>{-0.5, 0, -0.866}.data());

                //add motion//

                //leg1//
                auto& leg1_m1 = hex->addMotion(leg1_p2); //X副
                auto& leg1_m2 = hex->addMotion(leg1_p1); //Y副
                auto& leg1_m3 = hex->addMotion(leg1_r1); //R副

                //leg2//
                auto& leg2_m1 = hex->addMotion(leg2_p2); //X副
                auto& leg2_m2 = hex->addMotion(leg2_p1); //Y副
                auto& leg2_m3 = hex->addMotion(leg2_r1); //R副

                //leg3//
                auto& leg3_m1 = hex->addMotion(leg3_p2); //X副
                auto& leg3_m2 = hex->addMotion(leg3_p1); //Y副
                auto& leg3_m3 = hex->addMotion(leg3_r1); //R副

                //leg4//
                auto& leg4_m1 = hex->addMotion(leg4_p2); //X副
                auto& leg4_m2 = hex->addMotion(leg4_p1); //Y副
                auto& leg4_m3 = hex->addMotion(leg4_r1); //R副

                //leg5//
                auto& leg5_m1 = hex->addMotion(leg5_p2); //X副
                auto& leg5_m2 = hex->addMotion(leg5_p1); //Y副
                auto& leg5_m3 = hex->addMotion(leg5_r1); //R副

                //leg6//
                auto& leg6_m1 = hex->addMotion(leg6_p2); //X副
                auto& leg6_m2 = hex->addMotion(leg6_p1); //Y副
                auto& leg6_m3 = hex->addMotion(leg6_r1); //R副

                //add end-effector//
                auto body_ee_maki = body.addMarker("body_ee_mak_i");
                auto body_ee_makj = hex->ground().addMarker("body_ee_mak_j");

                auto& body_ee = hex->generalMotionPool().add<aris::dynamic::GeneralMotion>("body_ee", &body_ee_maki, &body_ee_makj);
                auto& leg1_ee = hex->addPointMotion(leg1_longest_bar, hex->ground(), ee_pos[0]);
                hex->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());  //这个地方是height吗
                auto& leg2_ee = hex->addPointMotion(leg2_longest_bar, hex->ground(), ee_pos[1]);
                hex->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
                auto& leg3_ee = hex->addPointMotion(leg3_longest_bar, hex->ground(), ee_pos[2]);
                hex->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
                auto& leg4_ee = hex->addPointMotion(leg4_longest_bar, hex->ground(), ee_pos[3]);
                hex->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
                auto& leg5_ee = hex->addPointMotion(leg5_longest_bar, hex->ground(), ee_pos[4]);
                hex->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());
                auto& leg6_ee = hex->addPointMotion(leg6_longest_bar, hex->ground(), ee_pos[5]);
                hex->ground().markerPool().back().setPrtPe(std::array<double, 6>{0, 0, 0, 0, 0, 0}.data());



                auto& inverse_kinematic_solver = hex->solverPool().add<aris::dynamic::InverseKinematicSolver>();
                auto& forward_kinematic_solver = hex->solverPool().add<aris::dynamic::ForwardKinematicSolver>();
                auto& inverse_dynamic_solver = hex->solverPool().add<aris::dynamic::InverseDynamicSolver>();
                auto& forward_dynamic_solver = hex->solverPool().add<aris::dynamic::ForwardDynamicSolver>();

                auto& stand_universal = hex->solverPool().add<aris::dynamic::UniversalSolver>();

                //添加仿真器和仿真结果//
                auto& adams = hex->simulatorPool().add<aris::dynamic::AdamsSimulator>();
                auto& result = hex->simResultPool().add<aris::dynamic::SimResult>();

                hex->init();

                // 设置默认拓扑结构 //
                for (auto& m : hex->motionPool())m.activate(true);
                for (auto& gm : hex->generalMotionPool())gm.activate(false);

                return hex;

            }






auto createControllerHexapod()->std::unique_ptr<aris::control::Controller>
{
    std::unique_ptr<aris::control::Controller> controller(new aris::control::EthercatController);

    for (aris::Size i = 0; i < 1; ++i)
    {
#ifdef ARIS_USE_ETHERCAT_SIMULATION
        double pos_offset[18]
        {
            0,0,0,
            0,0,0,
            0,0,0,
            0,0,0,
            0,0,0,
            0,0,0
        };
#else
        double pos_offset[18]
        {
            0,0,0,
            0,0,0,
            0,0,0,
            0,0,0,
            0,0,0,
            0,0,0

        };
#endif
        double pos_factor[18] //偏置系数
        {
            2000/PI,2000/PI,2000/PI,
            2000/PI,2000/PI,2000/PI,
            2000/PI,2000/PI,2000/PI,
            2000/PI,2000/PI,2000/PI,
            2000/PI,2000/PI,2000/PI,
            2000/PI,2000/PI,2000/PI
        };
        double max_pos[18] //最大位置
        {
            500*PI,500*PI,500*PI,
            500*PI,500*PI,500*PI,
            500*PI,500*PI,500*PI,
            500*PI,500*PI,500*PI,
            500*PI,500*PI,500*PI,
            500*PI,500*PI,500*PI
        };
        double min_pos[18] //最小位置
        {
            -500*PI,-500*PI,-500*PI,
            -500*PI,-500*PI,-500*PI,
            -500*PI,-500*PI,-500*PI,
            -500*PI,-500*PI,-500*PI,
            -500*PI,-500*PI,-500*PI,
            -500*PI,-500*PI,-500*PI
        };
        double max_vel[18]  //最大速度
        {
            100 * PI, 100 * PI,  100 * PI,
            330 / 60 * 2 * PI, 330 / 60 * 2 * PI,  330 / 60 * 2 * PI,
            330 / 60 * 2 * PI, 330 / 60 * 2 * PI,  330 / 60 * 2 * PI,
            330 / 60 * 2 * PI, 330 / 60 * 2 * PI,  330 / 60 * 2 * PI,
            330 / 60 * 2 * PI, 330 / 60 * 2 * PI,  330 / 60 * 2 * PI,
            330 / 60 * 2 * PI, 330 / 60 * 2 * PI,  330 / 60 * 2 * PI
        };
        double max_acc[18]  //最大加速度
        {
            3000,  3000,  3000,
            3000,  3000,  3000,
            3000,  3000,  3000,
            3000,  3000,  3000,
            3000,  3000,  3000,
            3000,  3000,  3000
        };

        int phy_id[18]={0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17};


        std::string xml_str =
            "<EthercatMotor phy_id=\"" + std::to_string(phy_id[i]) + "\" product_code=\"0x00\""
            " vendor_id=\"0x00\" revision_num=\"0x00\" dc_assign_activate=\"0x0300\""
            " min_pos=\"" + std::to_string(min_pos[i]) + "\" max_pos=\"" + std::to_string(max_pos[i]) + "\" max_vel=\"" + std::to_string(max_vel[i]) + "\" min_vel=\"" + std::to_string(-max_vel[i]) + "\""
            " max_acc=\"" + std::to_string(max_acc[i]) + "\" min_acc=\"" + std::to_string(-max_acc[i]) + "\" max_pos_following_error=\"10.0\" max_vel_following_error=\"20.0\""
            " home_pos=\"0\" pos_factor=\"" + std::to_string(pos_factor[i]) + "\" pos_offset=\"" + std::to_string(pos_offset[i]) + "\">"
            "	<SyncManagerPoolObject>"
            "		<SyncManager is_tx=\"false\"/>"
            "		<SyncManager is_tx=\"true\"/>"
            "		<SyncManager is_tx=\"false\">"
            "			<Pdo index=\"0x1600\" is_tx=\"false\">"
            "				<PdoEntry name=\"target_pos\" index=\"0x607A\" subindex=\"0x00\" size=\"32\"/>"
            "				<PdoEntry name=\"target_vel\" index=\"0x60FF\" subindex=\"0x00\" size=\"32\"/>"
//            "				<PdoEntry name=\"targer_toq\" index=\"0x6071\" subindex=\"0x00\" size=\"16\"/>"
//            "				<PdoEntry name=\"max_toq\" index=\"0x6072\" subindex=\"0x00\" size=\"16\"/>"
            "				<PdoEntry name=\"control_word\" index=\"0x6040\" subindex=\"0x00\" size=\"16\"/>"
            "				<PdoEntry name=\"mode_of_operation\" index=\"0x6060\" subindex=\"0x00\" size=\"8\"/>"
            "			</Pdo>"
            "		</SyncManager>"
            "		<SyncManager is_tx=\"true\">"
            "			<Pdo index=\"0x1A00\" is_tx=\"true\">"
            "				<PdoEntry name=\"status_word\" index=\"0x6041\" subindex=\"0x00\" size=\"16\"/>"
            "				<PdoEntry name=\"mode_of_display\" index=\"0x6061\" subindex=\"0x00\" size=\"8\"/>"
            "				<PdoEntry name=\"pos_actual_value\" index=\"0x6064\" subindex=\"0x00\" size=\"32\"/>"
            "				<PdoEntry name=\"vel_actual_value\" index=\"0x606c\" subindex=\"0x00\" size=\"32\"/>"
//            "				<PdoEntry name=\"toq_actual_value\" index=\"0x6077\" subindex=\"0x00\" size=\"16\"/>"
            "				<PdoEntry name=\"digital_inputs\" index=\"0x60FD\" subindex=\"0x00\" size=\"32\"/>"
            "			</Pdo>"
            "		</SyncManager>"
            "	</SyncManagerPoolObject>"
            "</EthercatMotor>";


        auto &s = controller->slavePool().add<aris::control::EthercatMotor>();
        aris::core::fromXmlString(s,xml_str);

#ifdef WIN32
        dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).setVirtual(true);
#endif

#ifndef WIN32
        dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).scanInfoForCurrentSlave();
#endif

        dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).setDcAssignActivate(0x300);
        dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).setControlWord(0x00);
        dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).setModeOfOperation(0x08);
        dynamic_cast<aris::control::EthercatMotor&>(controller->slavePool().back()).setTargetPos(0.0);
    };
    return controller;
}
auto createPlanHexapod()->std::unique_ptr<aris::plan::PlanRoot>
{
    std::unique_ptr<aris::plan::PlanRoot> plan_root(new aris::plan::PlanRoot);

    plan_root->planPool().add<aris::plan::Enable>();
    plan_root->planPool().add<aris::plan::Disable>();
    plan_root->planPool().add<aris::plan::Home>();
    plan_root->planPool().add<aris::plan::Mode>();
    plan_root->planPool().add<aris::plan::Show>();
    plan_root->planPool().add<aris::plan::Sleep>();
    plan_root->planPool().add<aris::plan::Clear>();
    plan_root->planPool().add<aris::plan::Recover>();
    auto &rs = plan_root->planPool().add<aris::plan::Reset>();
    rs.command().findParam("pos")->setDefaultValue("{0.5,0.392523364485981,0.789915966386555,0.5,0.5,0.5}");

    auto &mvaj = plan_root->planPool().add<aris::plan::MoveAbsJ>();
    mvaj.command().findParam("vel")->setDefaultValue("0.1");

    plan_root->planPool().add<aris::plan::MoveL>();
    plan_root->planPool().add<aris::plan::MoveJ>();
    plan_root->planPool().add<aris::plan::GetXml>();
    plan_root->planPool().add<aris::plan::SetXml>();
    plan_root->planPool().add<aris::plan::Start>();
    plan_root->planPool().add<aris::plan::Stop>();

    //驱动命令
    plan_root->planPool().add<Home>();


    //仿真命令
    plan_root->planPool().add<HexDynamicForwardTest>();
    plan_root->planPool().add<HexDynamicBackTest>();
    plan_root->planPool().add<HexDynamicRightTest>();
    plan_root->planPool().add<HexDynamicLeftTest>();
    plan_root->planPool().add<HexDynamicTurnRightTest>();
    plan_root->planPool().add<HexDynamicTurnLeftTest>();
    plan_root->planPool().add<HexDynamicTetrapodTest>();
    plan_root->planPool().add<HexWalkingPrmTest>();
    plan_root->planPool().add<HexLateralPrmTest>();
    plan_root->planPool().add<HexTurnPrmTest>();




    return plan_root;
}

}
