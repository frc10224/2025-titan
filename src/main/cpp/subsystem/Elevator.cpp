#include "subsystem/Elevator.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotController.h>

using namespace rev::spark;

Elevator::Elevator() :
	sysid_routine(frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt, nullptr},
        frc2::sysid::Mechanism{
            [this](units::volt_t driveVoltage) {
                leftMotor.SetVoltage(driveVoltage);
            },
            [this](frc::sysid::SysIdRoutineLog *log) {
				log->Motor("elevator-Left")
					.voltage(leftMotor.Get() *
								frc::RobotController::GetBatteryVoltage())
					.position(units::turn_t{leftMotor.GetEncoder().GetPosition()})
					.velocity(units::turns_per_second_t{leftMotor.GetEncoder().GetVelocity() / 60});
            },
            this
    }) {
    SparkBaseConfig config{};
    config.SetIdleMode(SparkBaseConfig::IdleMode::kCoast);
    config.closedLoop
        .P(ElevatorConstants::kP)
        .D(ElevatorConstants::kD)
        .VelocityFF(ElevatorConstants::kFF);

    leftMotor.Configure(config,
		SparkMax::ResetMode::kResetSafeParameters,
		SparkMax::PersistMode::kPersistParameters);

    config.Follow(rightMotor, true);

    rightMotor.Configure(config,
		SparkMax::ResetMode::kResetSafeParameters,
		SparkMax::PersistMode::kPersistParameters);
};

void Elevator::Periodic() {
    frc::SmartDashboard::PutNumber("Elevator/Velocity",
        leftMotor.GetEncoder().GetVelocity());
    frc::SmartDashboard::PutNumber("Elevator/Encoder_Position",
        leftMotor.GetEncoder().GetPosition());
};

void Elevator::SetPosition(double turns) {
    leftMotor.GetClosedLoopController()
        .SetReference(turns, SparkLowLevel::ControlType::kPosition);
}
