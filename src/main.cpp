#include"plan.h"
#include"kinematics.h"
#include<aris.hpp>
#include"robot.h"

extern double input_angle[18];
extern double file_current_leg[18];
extern double file_current_body[16];
int main(int argc, char *argv[])
{

	auto& cs = aris::server::ControlServer::instance();
	cs.resetController(robot::createControllerHexapod().release());
	cs.resetPlanRoot(robot::createPlanHexapod().release());
    cs.resetModel(robot::createModelHexapod().release());

	// 设置模型初始位置，给关节角度  注：  初始位置电机输入为0 //
	double set_init_position[18] = {
		0,0,0,
		0,0,0,
		0,0,0,
		0,0,0,
		0,0,0,
		0,0,0
	};


	

	cs.model().setInputPos(set_init_position); 
	if (cs.model().forwardKinematics()) THROW_FILE_LINE("forward failed"); //先算正解，得到末端位置


	auto& adams1 = dynamic_cast<aris::dynamic::Model&>(cs.model()).simulatorPool().front();
	//adams.saveadams("c:\\users\\jpche\\desktop\\aaa\\hexapod_simulation.cmd"); //这个只是导入模型
	

	//这些仿真运动并没有调用运动学反解，是通过aris库计算得来的解，如果要验证自己的解，将两者对比即可，此处无需纠结
	//robot::HexDynamicForwardTest plan;        //前进没有修改可以直接使用，运行时间4.5s
	//robot::HexDynamicBackTest plan;           //后退没有修改可以直接使用，运行时间4.5s
	//robot::HexDynamicRightTest plan;          //右移没有修改可以直接使用，运行时间4.5s
	//robot::HexDynamicLeftTest plan;           //左移没有修改可以直接使用，运行时间4.5s
	//robot::HexDynamicTurnRightTest plan;		//右转没有修改可以直接使用，运行时间4.5s，但步长只能是0.1，目前看来就是正负号导致不能更改步长,但问题未找到
	//robot::HexDynamicTurnLeftTest plan;		//左转没有修改可以直接使用，运行时间4.5s
	//robot::HexDynamicTetrapodTest plan;		//四足步态没有修改可以直接使用，运行时间6.5s
	robot::HexWalkingPrmTest plan;
	//robot::HexLateralPrmTest plan;
	//robot::HexTurnPrmTest plan;

	adams1.simulate(plan, dynamic_cast<aris::dynamic::Model&> (cs.model()).simResultPool().front());

	dynamic_cast<aris::dynamic::AdamsSimulator&>(adams1).saveAdams("C:\\Users\\jpche\\Desktop\\aaa\\hexapod_simulation_with_control.cmd", dynamic_cast<aris::dynamic::Model&>(cs.model()).simResultPool().front()); //这个模型还含有运动参数
	//aris::dynamic::Simulato
	//

	std::cout << "simulate finished" << std::endl;

	//等待终端输入函数，本函数不能去掉，否则实时线程和主线程都会结束//
	cs.init();

	//开启websocket/socket服务器//
	cs.open();
    cs.start();
	cs.runCmdLine();
	return 0;
}
