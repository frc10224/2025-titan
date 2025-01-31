#include <cmath>

#include <frc/MathUtil.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMax.h>

#include "subsystem/Drivetrain.h"
#include "subsystem/Pose.h"
#include "Constants.h"

#define PID_TUNE
#include "Bits.h"

using namespace rev::spark;

extern Pose *g_pose;

Motor::Motor(int id) :
		motor(id, SparkMax::MotorType::kBrushless),
		pid(motor.GetClosedLoopController()) {
	SparkBaseConfig config{};

	// we want the robot to stop when the drive releases the stick
	config.SetIdleMode(SparkBaseConfig::IdleMode::kBrake);

	using namespace DrivetrainConstants;
	config.closedLoop.Pidf(kP, kI, kD, kVelocityFF);

	motor.Configure(config,
		SparkMax::ResetMode::kResetSafeParameters,
		// if the controller power cycles we want it to remember these settings
		SparkMax::PersistMode::kPersistParameters);
}

Drivetrain::Drivetrain() {
	using namespace DrivetrainConstants;
	pid_tune_init("Drivetrain", TUNE_P | TUNE_D | TUNE_FF, kP, kI, kD, kVelocityFF);
}

void Drivetrain::Periodic() {
	g_pose->UpdateFromWheelPositions(GetWheelPositions());
	frc::SmartDashboard::PutNumber("Drivetrain/Motor/Lf_RPM", motorLf.GetEncoderSpeed());
	frc::SmartDashboard::PutNumber("Drivetrain/Motor/Lb_RPM", motorLb.GetEncoderSpeed());
	frc::SmartDashboard::PutNumber("Drivetrain/Motor/Rf_RPM", motorRf.GetEncoderSpeed());
	frc::SmartDashboard::PutNumber("Drivetrain/Motor/Rb_RPM", motorRb.GetEncoderSpeed());

	pid_tune(motorLf, TUNE_P | TUNE_D | TUNE_FF, "Drivetrain");
	pid_tune(motorLb, TUNE_P | TUNE_D | TUNE_FF, "Drivetrain");
	pid_tune(motorRf, TUNE_P | TUNE_D | TUNE_FF, "Drivetrain");
	pid_tune(motorRb, TUNE_P | TUNE_D | TUNE_FF, "Drivetrain");
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