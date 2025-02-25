#include "subsystem/Elevator.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/Commands.h>
#include <frc/RobotController.h>

using namespace rev::spark;

Elevator::Elevator() : setpoint(0), 
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

    config.softLimit.ReverseSoftLimitEnabled(true);
    config.softLimit.ForwardSoftLimitEnabled(false);

    config.softLimit.ReverseSoftLimit(0);
    //config.softLimit.ForwardSoftLimit(ElevatorConstants::kTopLimitSpinCount);

    leftMotor.Configure(config,
		SparkMax::ResetMode::kResetSafeParameters,
		SparkMax::PersistMode::kPersistParameters);

    config.softLimit.ForwardSoftLimitEnabled(true);
    config.softLimit.ReverseSoftLimitEnabled(false);
    config.softLimit.ForwardSoftLimit(0);
    //config.softLimit.ReverseSoftLimit(-ElevatorConstants::kTopLimitSpinCount);

    config.Follow(leftMotor, true);

    rightMotor.Configure(config,
		SparkMax::ResetMode::kResetSafeParameters,
		SparkMax::PersistMode::kPersistParameters);

    frc::SmartDashboard::PutNumber("Elevator/Compensation", 0);
};

void Elevator::Periodic() {
    frc::SmartDashboard::PutNumber("Elevator/Velocity",
        leftMotor.GetEncoder().GetVelocity());
    frc::SmartDashboard::PutNumber("Elevator/LeftEncoder_Position",
        leftMotor.GetEncoder().GetPosition());
    frc::SmartDashboard::PutNumber("Elevator/RightEncoder_Position",
        rightMotor.GetEncoder().GetPosition());
};

frc2::CommandPtr Elevator::SetPosition(double turns) {
    return frc2::cmd::StartEnd([this, turns] {
            leftMotor.GetClosedLoopController()
                .SetReference(turns + frc::SmartDashboard::GetNumber("Elevator/Compensation", 0), SparkLowLevel::ControlType::kPosition);
        },
        [this] {
            leftMotor.Set(0);
        }
    );
}


frc2::CommandPtr Elevator::MoveUp(double turns) {
    return frc2::cmd::RunOnce([this, turns] {
            setpoint += turns;
            leftMotor.GetClosedLoopController()
                .SetReference(setpoint, SparkLowLevel::ControlType::kPosition);
        }, {this}
    );
}