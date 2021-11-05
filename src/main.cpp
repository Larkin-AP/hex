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

	// 设置模型初始位置，给关节角度  注：相对的位置是模型Quad里设置的关节轴和末端  初始位置电机输入为0 //
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
	//robot::setStandTopologyIK(cs);
	
	auto& adams1 = dynamic_cast<aris::dynamic::AdamsSimulator&>(cs.model().simulatorPool().front());
	auto& adams2 = dynamic_cast<aris::dynamic::AdamsSimulator&>(cs.model().simulatorPool().front());
	//adams.saveAdams("C:\\Users\\jpche\\Desktop\\aaa\\hexapod_simulation.cmd"); //这个只是导入模型
	

	robot::HexDynamicForwardTest plan;
	//robot::HexDynamicBackTest plan;
	//robot::HexDynamicRightTest plan;
	//robot::HexDynamicLeftTest plan;
	//robot::HexDynamicTurnRightTest plan;
	//robot::HexDynamicTurnLeftTest plan;
	//robot::HexDynamicTetrapodTest plan;

	adams1.simulate(plan, cs.model().simResultPool().front());
	adams1.saveAdams("C:\\Users\\jpche\\Desktop\\aaa\\hexapod_simulation_with_control.cmd", cs.model().simResultPool().front()); //这个模型还含有运动参数

	

	std::cout << "simulate finished" << std::endl;

	//等待终端输入函数，本函数不能去掉，否则实时线程和主线程都会结束//
	cs.init();

	//开启websocket/socket服务器//
	cs.open();
	cs.runCmdLine();
	return 0;
}
