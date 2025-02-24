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

// #define PID_TUNE

using namespace rev::spark;

extern Pose *g_pose;

Motor::Motor(int id) :
		motor(id, SparkMax::MotorType::kBrushless),
		pid(motor.GetClosedLoopController()) {
	SparkBaseConfig config{};

	// PID should stop fine for us
	config.SetIdleMode(SparkBaseConfig::IdleMode::kCoast);
	
	config.closedLoop
		.P(DrivetrainConstants::kP)
		.I(DrivetrainConstants::kI)
		.D(DrivetrainConstants::kD)
		.VelocityFF(DrivetrainConstants::kVelocityFF);
	
	config.encoder.PositionConversionFactor(1);

	motor.Configure(config,
		SparkMax::ResetMode::kResetSafeParameters,
		// if the controller power cycles we want it to remember these settings
		SparkMax::PersistMode::kPersistParameters);
}

void Motor::Log(frc::sysid::SysIdRoutineLog *log, const char *title) {
	log->Motor(title)
		.voltage(motor.Get() *
					frc::RobotController::GetBatteryVoltage())
		.position(units::turn_t{motor.GetEncoder().GetPosition()})
		.velocity(GetEncoderVelocity());
}

Drivetrain::Drivetrain() :
	sysid_routine(frc2::sysid::Config{std::nullopt, std::nullopt, 3_s, nullptr},
        frc2::sysid::Mechanism{
            [this](units::volt_t driveVoltage) {
                motorLf.SetVoltage(-driveVoltage);
                motorLb.SetVoltage(-driveVoltage);
                motorRf.SetVoltage(driveVoltage);
                motorRb.SetVoltage(driveVoltage);
            },
            [this](frc::sysid::SysIdRoutineLog *log) {
                motorLf.Log(log, "drive-leftFront");
                motorLb.Log(log, "drive-leftBack");
                motorRf.Log(log, "drive-rightFront");
                motorRb.Log(log, "drive-rightBack");
            },
            this
    }) {}

void Drivetrain::Periodic() {
	g_pose->UpdateFromWheelPositions(GetWheelPositions());
	frc::SmartDashboard::PutNumber("Drivetrain/Motor/Lf_RPS", (double)motorLf.GetEncoderVelocity());
	frc::SmartDashboard::PutNumber("Drivetrain/Motor/Lb_RPS", (double)motorLb.GetEncoderVelocity());
	frc::SmartDashboard::PutNumber("Drivetrain/Motor/Rf_RPS", (double)motorRf.GetEncoderVelocity());
	frc::SmartDashboard::PutNumber("Drivetrain/Motor/Rb_RPS", (double)motorRb.GetEncoderVelocity());
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
						
		motorLf.SetVelocity(-lf);
		motorRf.SetVelocity(rf);
		motorLb.SetVelocity(-lb);
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