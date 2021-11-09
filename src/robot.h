#ifndef ROBOT_H_
#define ROBOT_H_

#include<memory>
#include<aris.hpp>

namespace robot
{
	//pos_offset��¼���Ǽ���λ���ϵ��prepareλ�ö�Ӧ�ĵ��λ��
	static const double pos_offset[18] = {
			0,0,0,
			0,0,0,
			0,0,0,
			0,0,0,
			0,0,0,
			0,0,0
	};

	//---------------------��ȡ��ǰ�����λ��--------------------//
	class ReadCurrentPos :public aris::core::CloneObject<ReadCurrentPos, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		virtual ~ReadCurrentPos();
		explicit ReadCurrentPos(const std::string& name = "Read");
	};

	//---------------------�Ӽ���λ���ƶ���prepareλ��----------------//
		//�ڼ���λ���ϵ磬ƫת�̶��Ƕȣ�����prepareλ�ã����λ�ÿɽ�������ƫת����λ�ã�������ʱ�����λ�ã�Ȼ�����벢��ס��Ӧ�ÿ��Թ̶�����ֵ
	class Prepare :public aris::core::CloneObject<Prepare, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		virtual ~Prepare();
		explicit Prepare(const std::string& name = "prepare");

	};

	//---------------------������λ���ƶ���prepareλ��----------------//
	//����λ���ϵ磬���������Ҳ���Իص�prepareλ��
	class Home :public aris::core::CloneObject<Home, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		virtual ~Home();
		explicit Home(const std::string& name = "home");

	};

	//-------------------ÿ����������ܲ��ԣ����������ƶ���-------------------//
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

	//-------------------ÿ����������ܲ��ԣ����������ƶ���-------------------//
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


	//-------------------����������ܲ��ԣ����������ƶ���-------------------//
	class MoveJointSingle :public aris::core::CloneObject<MoveJointSingle, aris::plan::Plan>
	{
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int;
		auto virtual collectNrt()->void;

		virtual ~MoveJointSingle();
		explicit MoveJointSingle(const std::string& name = "moveJS");
	private:
		double cef_[18];
	};

	//-------------------����   ǰ������-------------------//
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

	//-------------------����   �����ƶ�-------------------//
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

	//-------------------����   ����ת��-------------------//
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

	//-------------------����   ���㲽̬-------------------//
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

	//--------------------����TCurve2-----------------------//
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
	//ǰ��
	class HexDynamicForwardTest :public aris::core::CloneObject<HexDynamicForwardTest, aris::plan::Plan> {
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int override;

		virtual ~HexDynamicForwardTest();
		explicit HexDynamicForwardTest(const std::string& name = "hex_forward");
	};
	//����
	class HexDynamicBackTest :public aris::core::CloneObject<HexDynamicBackTest, aris::plan::Plan> {
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int override;

		virtual ~HexDynamicBackTest();
		explicit HexDynamicBackTest(const std::string& name = "hex_back");
	};
	//����
	class HexDynamicRightTest :public aris::core::CloneObject<HexDynamicRightTest, aris::plan::Plan> {
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int override;

		virtual ~HexDynamicRightTest();
		explicit HexDynamicRightTest(const std::string& name = "hex_right");
	};
	//����
	class HexDynamicLeftTest :public aris::core::CloneObject<HexDynamicLeftTest, aris::plan::Plan> {
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int override;

		virtual ~HexDynamicLeftTest();
		explicit HexDynamicLeftTest(const std::string& name = "hex_left");
	};
	//��ת
	class HexDynamicTurnRightTest :public aris::core::CloneObject<HexDynamicTurnRightTest, aris::plan::Plan> {
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int override;

		virtual ~HexDynamicTurnRightTest();
		explicit HexDynamicTurnRightTest(const std::string& name = "hex_turn_right");
	};

	//��ת
	class HexDynamicTurnLeftTest :public aris::core::CloneObject<HexDynamicTurnLeftTest, aris::plan::Plan> {
	public:
		auto virtual prepareNrt()->void;
		auto virtual executeRT()->int override;

		virtual ~HexDynamicTurnLeftTest();
		explicit HexDynamicTurnLeftTest(const std::string& name = "hex_turn_left");
	};

	//���㲽̬
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
	auto setStandTopologyIK(aris::server::ControlServer& cs)->void;
}
#endif