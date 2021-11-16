#ifndef ROBOT_H_
#define ROBOT_H_

#include<memory>
#include<aris.hpp>

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

	//---------------------读取当前电机的位置--------------------//
	class ReadCurrentPos :public aris::core::CloneObject<ReadCurrentPos, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		virtual ~ReadCurrentPos();
		explicit ReadCurrentPos(const std::string& name = "Read");
	};


    //---------------------从任意位置移动到prepare位置,在极限位置上电----------------//
    //极限位置上电，
	class Home :public aris::core::CloneObject<Home, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		virtual ~Home();
		explicit Home(const std::string& name = "home");

	};

    //---------------------从任意位置移动到prepare位置----------------//
    //在零位上电
    class Home2 :public aris::core::CloneObject<Home2, aris::plan::Plan>
    {
    public:
        auto virtual prepareNrt()->void;
        auto virtual executeRT()->int;
        auto virtual collectNrt()->void;

        virtual ~Home2();
        explicit Home2(const std::string& name = "home2");

    };

	//-------------------每个电机简单性能测试（梯形曲线移动）-------------------//
		class MoveJointAll :public aris::core::CloneObject<MoveJointAll, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		virtual ~MoveJointAll();
		explicit MoveJointAll(const std::string& name = "moveJA");
	private:
		double cef_;
		
	};

	//-------------------每个电机简单性能测试（梯形曲线移动）-------------------//
	class MoveJointAllCos :public aris::core::CloneObject<MoveJointAllCos, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		virtual ~MoveJointAllCos();
		explicit MoveJointAllCos(const std::string& name = "moveJAC");
	private:
		double cef_;

	};



	//-------------------驱动   前进后退-------------------//
	class HexForward :public aris::core::CloneObject<HexForward, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		virtual ~HexForward();
		explicit HexForward(const std::string& name = "forward");
	private:
		double x_step_;
		int n_;
	};

	//-------------------驱动   左右移动-------------------//
	class HexLateral :public aris::core::CloneObject<HexLateral, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		virtual ~HexLateral();
		explicit HexLateral(const std::string& name = "lateral");
	private:
		double z_step_;
		int n_;
	};

	//-------------------驱动   左右转动-------------------//
	class HexTurn :public aris::core::CloneObject<HexTurn, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		virtual ~HexTurn();
		explicit HexTurn(const std::string& name = "turn");
	private:
		double turn_yaw_;
		int n_;
	};

	//-------------------驱动   四足步态-------------------//
	class HexTetrapod :public aris::core::CloneObject<HexTetrapod, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		virtual ~HexTetrapod();
		explicit HexTetrapod(const std::string& name = "turn");
	private:
		double x_step_;
		int n_;
	};

	//--------------------测试TCurve2-----------------------//
	class TCurve2Test :public aris::core::CloneObject<TCurve2Test, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		virtual ~TCurve2Test();
		explicit TCurve2Test(const std::string& name = "test");

	};


	


	class HexRead :public aris::core::CloneObject<HexRead, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;

		virtual ~HexRead();
		explicit HexRead(const std::string& name = "HexRead");
	private:
		double turn_angle_;
	};
	//前进
	class HexDynamicForwardTest :public aris::core::CloneObject<HexDynamicForwardTest, aris::plan::Plan> {
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int override;

		virtual ~HexDynamicForwardTest();
		explicit HexDynamicForwardTest(const std::string& name = "hex_forward");
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

}
#endif
