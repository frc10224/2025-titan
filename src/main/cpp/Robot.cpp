#include <frc/TimedRobot.h>

#include <frc2/command/CommandScheduler.h>
#include <frc2/command/CommandPtr.h>

#include "RobotContainer.h"

class Robot : public frc::TimedRobot {
private:
	RobotContainer container{};
	frc2::CommandPtr teleopCmd{container.CreateTeleopCommand()};
	frc2::CommandPtr autoCmd{container.CreateAutoCommand()};
public:
	void RobotInit() override {}
	
	// This functions runs once every 20ms.
	void RobotPeriodic() override {
		frc2::CommandScheduler::GetInstance().Run();
	}

	void DisabledInit() override {};
	void DisabledPeriodic() override {};
	void DisabledExit() override {};

	void AutonomousInit() override {};
	void AutonomousPeriodic() override {
		autoCmd.Schedule();
	};
	void AutonomousExit() override {};

	void TeleopInit() override {};
	void TeleopPeriodic() override {
		teleopCmd.Schedule();
	};
	void TeleopExit() override {};

	void TestInit() override {};
	void TestPeriodic() override {};
	void TestExit() override {};

	void SimulationInit() override {};
	void SimulationPeriodic() override {};
};

#ifndef RUNNING_FRC_TESTS
int main() {
	return frc::StartRobot<Robot>();
}
#endif
