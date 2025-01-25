#include "subsystem/Elevator.h"

#include <frc/smartdashboard/SmartDashboard.h>

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
};

void Elevator::Periodic() {
    frc::SmartDashboard::PutNumber("Elevator/Velocity",
        leftMotor.GetEncoder().GetVelocity());
    frc::SmartDashboard::PutNumber("Elevator/Encoder Position",
        leftMotor.GetEncoder().GetPosition());
};

void Elevator::SetPosition(double turns) {
    leftMotor.GetClosedLoopController()
        .SetReference(turns, SparkLowLevel::ControlType::kPosition);
}
