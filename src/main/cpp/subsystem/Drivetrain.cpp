#include <cmath>

#include <frc/MathUtil.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
// #include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMax.h>

#include "subsystem/Drivetrain.h"
#include "subsystem/Pose.h"
#include "Constants.h"
#include "Bits.h"

using namespace rev::spark;

extern Pose *g_pose;

Drivetrain::Drivetrain() {
	const auto pm = SparkMax::PersistMode::kPersistParameters;
	const auto rm = SparkMax::ResetMode::kResetSafeParameters;

	rev::spark::SparkBaseConfig config{};
	config.SetIdleMode(SparkBaseConfig::IdleMode::kBrake);

	motorLb.Configure(config, rm, pm);
	motorLf.Configure(config, rm, pm);
	motorRb.Configure(config, rm, pm);
	motorRf.Configure(config, rm, pm);
}

void Drivetrain::Periodic() {
	g_pose->UpdateFromWheelPositions(GetWheelPositions());
}

frc2::CommandPtr Drivetrain::MecanumDrive(Fn<double> XSpeed, Fn<double> YSpeed, Fn<double> ZRotate) {
	return frc2::cmd::Run([this, XSpeed, YSpeed, ZRotate] {
		double xSpeed = XSpeed();
		double ySpeed = YSpeed();
		double zRotate = ZRotate();

		xSpeed *= DrivetrainConstants::kMaxDriveSpeed;
		ySpeed *= DrivetrainConstants::kMaxDriveSpeed;
		zRotate *= DrivetrainConstants::kMaxTurnSpeed;

		frc::ApplyDeadband(xSpeed, 0.1);
		frc::ApplyDeadband(ySpeed, 0.1);
		frc::ApplyDeadband(zRotate, 0.1);

		drive.DriveCartesian(xSpeed, ySpeed, zRotate, g_pose->GyroAngle());
	}, {this});
}

frc::MecanumDriveWheelPositions Drivetrain::GetWheelPositions() {
	units::meter_t wheelPos[4] = {
		DrivetrainConstants::kWheelRadius * M_PI * motorLf.GetEncoder().GetPosition(),
		DrivetrainConstants::kWheelRadius * M_PI * motorRf.GetEncoder().GetPosition(),
		DrivetrainConstants::kWheelRadius * M_PI * motorLb.GetEncoder().GetPosition(),
		DrivetrainConstants::kWheelRadius * M_PI * motorRb.GetEncoder().GetPosition(),
	};
	// ...please don't break
	return *(frc::MecanumDriveWheelPositions *)&wheelPos;
}