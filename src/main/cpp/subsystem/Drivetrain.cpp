#include <frc/MathUtil.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>
// #include <frc/smartdashboard/SmartDashboard.h>
#include <rev/SparkMax.h>

#include "subsystem/Drivetrain.h"
#include "Constants.h"
#include "Bits.h"

using namespace rev::spark;

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
	// put smartdash infos here etc
}

// long type name
frc2::CommandPtr Drivetrain::MecanumDrive(Fn<double> xSpeed, Fn<double> ySpeed, Fn<double> zRotate) {
	class MecanumDriveCmd : public frc2::CommandHelper<frc2::Command, MecanumDriveCmd> {
		Fn<double> GetXSpeed, GetYSpeed, GetZRotate;
		Drivetrain *drivetrain;
	public:
		MecanumDriveCmd(Fn<double> x, Fn<double> y, Fn<double> z, Drivetrain *d)
			: GetXSpeed(x), GetYSpeed(y), GetZRotate(z), drivetrain(d) {};
		void execute() {
			double xSpeed = GetXSpeed();
			double ySpeed = GetYSpeed();
			double zRotate = GetZRotate();

			xSpeed *= MAX_DRIVE_SPEED;
			ySpeed *= MAX_DRIVE_SPEED;
			zRotate *= MAX_TURN_SPEED;

			frc::ApplyDeadband(xSpeed, 0.1);
			frc::ApplyDeadband(ySpeed, 0.1);
			frc::ApplyDeadband(zRotate, 0.1);

			drivetrain->m_robotDrive.DriveCartesian(GetXSpeed(), GetYSpeed(), GetZRotate());
		}
	};
	return MecanumDriveCmd(xSpeed, ySpeed, zRotate, this).ToPtr();
}
#undef DS