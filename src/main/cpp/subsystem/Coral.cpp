#include "subsystem/Coral.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

#include "Bits.h"

using namespace rev::spark;

Coral::Coral() {
	SparkBaseConfig config{};
	config.SetIdleMode(SparkBaseConfig::IdleMode::kBrake);
	config.closedLoop
		.P(CoralConstants::kP)
		.D(CoralConstants::kD);

    leftMotor.Configure(config,
		SparkMax::ResetMode::kResetSafeParameters,
		SparkMax::PersistMode::kPersistParameters);

    config.Follow(rightMotor, true);

    rightMotor.Configure(config,
		SparkMax::ResetMode::kResetSafeParameters,
		SparkMax::PersistMode::kPersistParameters);

	laser.set_ranging_mode(grpl::LaserCanRangingMode::Short);
	laser.set_timing_budget(grpl::LaserCanTimingBudget::TB33ms);
};

void Coral::Periodic() {
    frc::SmartDashboard::PutNumber("Coral RPM",
        leftMotor.GetEncoder().GetVelocity());
};

frc2::CommandPtr Coral::Collect() {
	return frc2::cmd::Either(
		frc2::cmd::Run([this] { rightMotor.Set(0.0); }),
		frc2::cmd::Run([this] { rightMotor.Set(0.5); }),
		[this] {
			std::optional<grpl::LaserCanMeasurement> measurement = laser.get_measurement();
			return measurement.has_value()
				&& measurement.value().distance_mm > 50;
		}
	);
}

frc2::CommandPtr Coral::Spit() {
	return frc2::cmd::Either(
		frc2::cmd::Run([this] { rightMotor.Set(0.5); }),
		frc2::cmd::Run([this] { rightMotor.Set(0.0); }),
		[this] {
			std::optional<grpl::LaserCanMeasurement> measurement = laser.get_measurement();
			return measurement.has_value()
				&& measurement.value().distance_mm > 50;
		}
	);

}