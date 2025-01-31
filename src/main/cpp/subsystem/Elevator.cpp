#include "subsystem/Elevator.h"

#include <frc/smartdashboard/SmartDashboard.h>

using namespace rev::spark;

#define PID_TUNE

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
#ifdef PID_TUNE
    frc::SmartDashboard::PutNumber("Elevator/PID/P", ElevatorConstants::kP);
	frc::SmartDashboard::PutNumber("Elevator/PID/D", ElevatorConstants::kD);
	frc::SmartDashboard::PutNumber("Elevator/PID/FF", ElevatorConstants::kFF);
#endif /* PID_TUNE */
};

void Elevator::Periodic() {
    frc::SmartDashboard::PutNumber("Elevator/Velocity",
        leftMotor.GetEncoder().GetVelocity());
    frc::SmartDashboard::PutNumber("Elevator/Encoder_Position",
        leftMotor.GetEncoder().GetPosition());
#ifdef PID_TUNE
	SparkBaseConfig config{};
	double p = frc::SmartDashboard::GetNumber("Elevator/PID/P", ElevatorConstants::kP);
	double d = frc::SmartDashboard::GetNumber("Elevator/PID/D", ElevatorConstants::kD);
	double ff = frc::SmartDashboard::GetNumber("Elevator/PID/FF", ElevatorConstants::kFF);
	config.closedLoop.Pidf(p, 0, d, ff);
	leftMotor.Configure(config,
		SparkMax::ResetMode::kNoResetSafeParameters,
		SparkMax::PersistMode::kNoPersistParameters);
	rightMotor.Configure(config,
		SparkMax::ResetMode::kNoResetSafeParameters,
		SparkMax::PersistMode::kNoPersistParameters);
#endif /* PID_TUNE */
};

void Elevator::SetPosition(double turns) {
    leftMotor.GetClosedLoopController()
        .SetReference(turns, SparkLowLevel::ControlType::kPosition);
}
