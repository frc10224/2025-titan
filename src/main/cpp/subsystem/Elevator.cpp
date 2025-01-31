#include "subsystem/Elevator.h"

#include <frc/smartdashboard/SmartDashboard.h>

#define PID_TUNE
#include "Bits.h"

using namespace rev::spark;

Elevator::Elevator() {
    SparkBaseConfig config{};
    config.SetIdleMode(SparkBaseConfig::IdleMode::kBrake);
    config.closedLoop
        .P(ElevatorConstants::kP)
        .D(ElevatorConstants::kD)
        .VelocityFF(ElevatorConstants::kFF);

    leftMotor.Configure(config,
		SparkMax::ResetMode::kResetSafeParameters,
		SparkMax::PersistMode::kPersistParameters);

    config.Follow(rightMotor);
    rightMotor.Configure(config,
		SparkMax::ResetMode::kResetSafeParameters,
		SparkMax::PersistMode::kPersistParameters);

	pid_tune_init("Elevator", TUNE_P | TUNE_D, 0, 0, 0, 0.01, "");
};

void Elevator::Periodic() {
    frc::SmartDashboard::PutNumber("Elevator/Velocity",
        leftMotor.GetEncoder().GetVelocity());
    frc::SmartDashboard::PutNumber("Elevator/Encoder_Position",
        leftMotor.GetEncoder().GetPosition());
	pid_tune(leftMotor, TUNE_P | TUNE_D, "Elevator");
};

void Elevator::SetPosition(double turns) {
    leftMotor.GetClosedLoopController()
        .SetReference(turns, SparkLowLevel::ControlType::kPosition);
}
