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
#include "Bits.h"

#define PID_TUNE

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

Drivetrain::Drivetrain() {
#ifdef PID_TUNE
	frc::SmartDashboard::PutNumber("Drivetrain/PID/P", DrivetrainConstants::kP);
	frc::SmartDashboard::PutNumber("Drivetrain/PID/I", DrivetrainConstants::kI);
	frc::SmartDashboard::PutNumber("Drivetrain/PID/D", DrivetrainConstants::kD);
	frc::SmartDashboard::PutNumber("Drivetrain/PID/FF", DrivetrainConstants::kVelocityFF);
#endif /* PID_TUNE */
}

void Drivetrain::Periodic() {
	g_pose->UpdateFromWheelPositions(GetWheelPositions());
	frc::SmartDashboard::PutNumber("Drivetrain/Motor/Lf_RPM", motorLf.GetEncoderSpeed());
	frc::SmartDashboard::PutNumber("Drivetrain/Motor/Lb_RPM", motorLb.GetEncoderSpeed());
	frc::SmartDashboard::PutNumber("Drivetrain/Motor/Rf_RPM", motorRf.GetEncoderSpeed());
	frc::SmartDashboard::PutNumber("Drivetrain/Motor/Rb_RPM", motorRb.GetEncoderSpeed());
#ifdef PID_TUNE
	SparkBaseConfig config{};
	double p = frc::SmartDashboard::GetNumber("Drivetrain/PID/P", DrivetrainConstants::kP);
	double i = frc::SmartDashboard::GetNumber("Drivetrain/PID/I", DrivetrainConstants::kI);
	double d = frc::SmartDashboard::GetNumber("Drivetrain/PID/D", DrivetrainConstants::kD);
	double ff = frc::SmartDashboard::GetNumber("Drivetrain/PID/FF", DrivetrainConstants::kVelocityFF);
	config.closedLoop.Pidf(p, i, d, ff);
	motorLf.motor.Configure(config,
		SparkMax::ResetMode::kNoResetSafeParameters,
		SparkMax::PersistMode::kNoPersistParameters);
	motorLb.motor.Configure(config,
		SparkMax::ResetMode::kNoResetSafeParameters,
		SparkMax::PersistMode::kNoPersistParameters);
	motorRf.motor.Configure(config,
		SparkMax::ResetMode::kNoResetSafeParameters,
		SparkMax::PersistMode::kNoPersistParameters);
	motorRb.motor.Configure(config,
		SparkMax::ResetMode::kNoResetSafeParameters,
		SparkMax::PersistMode::kNoPersistParameters);
#endif /* PID_TUNE */
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