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
};

void Coral::Periodic() {
    frc::SmartDashboard::PutNumber("Coral RPM",
        leftMotor.GetEncoder().GetVelocity());
};

frc2::CommandPtr Coral::Feed(Fn<double> Speed) {
	return frc2::cmd::RunEnd([this, Speed] {
        leftMotor.Set(Speed());
	}, [this] {leftMotor.Set(0);}, {this});
}