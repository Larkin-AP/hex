#ifndef ROBOT_H_
#define ROBOT_H_

#include<memory>
#include<aris.hpp>
#include"plan.h"
namespace robot
{
	//pos_offset记录的是极限位置上电后，prepare位置对应的电机位置
	static const double pos_offset[18] = {
            5,5,5,
			0,0,0,
			0,0,0,
			0,0,0,
			0,0,0,
			0,0,0
	};




	//----------------以下为仿真区域------------------------------//
	//前进
	class HexDynamicForwardTest :public aris::core::CloneObject<HexDynamicForwardTest, aris::plan::Plan> {
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int override;

		virtual ~HexDynamicForwardTest();
		explicit HexDynamicForwardTest(const std::string& name = "hex_forward");
	};


	//行走参数前进
	class HexWalkingPrmTest :public aris::core::CloneObject<HexWalkingPrmTest, aris::plan::Plan> {
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int override;

		virtual ~HexWalkingPrmTest();
		explicit HexWalkingPrmTest(const std::string& name = "walk_prm");
	};


	//行走参数侧移
	class HexLateralPrmTest :public aris::core::CloneObject<HexLateralPrmTest, aris::plan::Plan> {
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int override;

		virtual ~HexLateralPrmTest();
		explicit HexLateralPrmTest(const std::string& name = "lateral_prm");
	};

	//行走参数转弯
	class HexTurnPrmTest :public aris::core::CloneObject<HexTurnPrmTest, aris::plan::Plan> {
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int override;

		virtual ~HexTurnPrmTest();
		explicit HexTurnPrmTest(const std::string& name = "turn_prm");
	};



	//后退
	class HexDynamicBackTest :public aris::core::CloneObject<HexDynamicBackTest, aris::plan::Plan> {
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int override;

		virtual ~HexDynamicBackTest();
		explicit HexDynamicBackTest(const std::string& name = "hex_back");
	};
	//右移
	class HexDynamicRightTest :public aris::core::CloneObject<HexDynamicRightTest, aris::plan::Plan> {
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int override;

		virtual ~HexDynamicRightTest();
		explicit HexDynamicRightTest(const std::string& name = "hex_right");
	};
	//左移
	class HexDynamicLeftTest :public aris::core::CloneObject<HexDynamicLeftTest, aris::plan::Plan> {
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int override;

		virtual ~HexDynamicLeftTest();
		explicit HexDynamicLeftTest(const std::string& name = "hex_left");
	};
	//右转
	class HexDynamicTurnRightTest :public aris::core::CloneObject<HexDynamicTurnRightTest, aris::plan::Plan> {
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int override;

		virtual ~HexDynamicTurnRightTest();
		explicit HexDynamicTurnRightTest(const std::string& name = "hex_turn_right");
	};

	//左转
	class HexDynamicTurnLeftTest :public aris::core::CloneObject<HexDynamicTurnLeftTest, aris::plan::Plan> {
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int override;

		virtual ~HexDynamicTurnLeftTest();
		explicit HexDynamicTurnLeftTest(const std::string& name = "hex_turn_left");
	};

	//四足步态
	class HexDynamicTetrapodTest :public aris::core::CloneObject<HexDynamicTetrapodTest, aris::plan::Plan> {
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int override;

		virtual ~HexDynamicTetrapodTest();
		explicit HexDynamicTetrapodTest(const std::string& name = "hex_tetrapod");
	};


	auto createModelHexapod()->std::unique_ptr<aris::dynamic::Model>;
	auto createControllerHexapod()->std::unique_ptr<aris::control::Controller>;
	auto createPlanHexapod()->std::unique_ptr<aris::plan::PlanRoot>;
	auto CalculateInitPos()->void;
	auto CalculateLegEE(double* ee1,double* all_ee)->void;
	auto GenTrajToEE(double* all_ee, TCurve& s1, int count)->void; //count 是从0开始

}
#endif
