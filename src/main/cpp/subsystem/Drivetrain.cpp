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

Motor::Motor(int id) :
		motor(id, SparkMax::MotorType::kBrushless),
		pid(motor.GetClosedLoopController()) {
	SparkBaseConfig config{};

	// we want the robot to stop when the drive releases the stick
	config.SetIdleMode(SparkBaseConfig::IdleMode::kBrake);
	
	config.closedLoop
		.P(DrivetrainConstants::kP)
		.I(DrivetrainConstants::kI)
		.D(DrivetrainConstants::kD)
		.VelocityFF(DrivetrainConstants::kVelocityFF);
	
	motor.Configure(config,
		SparkMax::ResetMode::kResetSafeParameters,
		// if the controller power cycles we want it to remember these settings
		SparkMax::PersistMode::kPersistParameters);
}

Drivetrain::Drivetrain() {}

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

		auto [lf, rf, lb, rb] =
      		frc::MecanumDrive::DriveCartesianIK(xSpeed, ySpeed, zRotate, g_pose->GyroAngle());
		
		motorLf.SetVelocity(lf);
		motorRf.SetVelocity(rf);
		motorLb.SetVelocity(lb);
		motorRb.SetVelocity(rb);

	}, {this});
}

frc::MecanumDriveWheelPositions Drivetrain::GetWheelPositions() {
	return { 
		motorLf.GetWheelDistance(),
		motorRf.GetWheelDistance(),
		motorLb.GetWheelDistance(),
		motorRb.GetWheelDistance(),
	};
}