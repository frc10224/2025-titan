#include "subsystem/Coral.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/RobotController.h>
#include <frc2/command/CommandPtr.h>
#include <frc2/command/Commands.h>

#include <grpl/CanBridge.h>

#include "Bits.h"

using namespace rev::spark;

Coral::Coral() : sysid_routine(frc2::sysid::Config{std::nullopt, std::nullopt, std::nullopt, nullptr},
        frc2::sysid::Mechanism{
            [this](units::volt_t driveVoltage) {
                leftMotor.SetVoltage(driveVoltage);
            },
            [this](frc::sysid::SysIdRoutineLog *log) {
				log->Motor("coral-Left")
					.voltage(leftMotor.Get() *
								frc::RobotController::GetBatteryVoltage())
					.position(units::turn_t{leftMotor.GetEncoder().GetPosition()})
					.velocity(units::turns_per_second_t{leftMotor.GetEncoder().GetVelocity() / 60});
            },
            this
	})  {
	grpl::start_can_bridge();

	SparkBaseConfig config{};
	config.SetIdleMode(SparkBaseConfig::IdleMode::kBrake);
	config.closedLoop
		.P(CoralConstants::kP)
		.D(CoralConstants::kD)
		.VelocityFF(CoralConstants::kFF);

    leftMotor.Configure(config,
		SparkMax::ResetMode::kResetSafeParameters,
		SparkMax::PersistMode::kPersistParameters);

    config.Follow(leftMotor, true);

    rightMotor.Configure(config,
		SparkMax::ResetMode::kResetSafeParameters,
		SparkMax::PersistMode::kPersistParameters);

	laser.set_ranging_mode(grpl::LaserCanRangingMode::Short);
	laser.set_timing_budget(grpl::LaserCanTimingBudget::TB33ms);
};

void Coral::Periodic() {
    frc::SmartDashboard::PutNumber("Coral/RPM",
        leftMotor.GetEncoder().GetVelocity());
	
	laser_measurement = laser.get_measurement();
	if_hot (laser_measurement.has_value())
		frc::SmartDashboard::PutNumber("Laser distance (mm)",
			laser_measurement.value().distance_mm);
	else
		frc::SmartDashboard::PutString("Laser distance (mm)", "no value!");
};

void Coral::SetVelocity(double rpm) {
	leftMotor.GetClosedLoopController()
		.SetReference(rpm, rev::spark::SparkLowLevel::ControlType::kVelocity);
}

frc2::CommandPtr Coral::Collect() {
	return frc2::cmd::RunEnd(
		[this] {
			if (laser_measurement.has_value() && laser_measurement.value().distance_mm > 10)
				SetVelocity(150);
			else
				SetVelocity(0);
		},
		[this] {
			SetVelocity(0);
		}
	);
}

frc2::CommandPtr Coral::Spit() {
	return frc2::cmd::RunEnd(
		[this] {
			if (laser_measurement.has_value() && laser_measurement.value().distance_mm > 10)
				SetVelocity(0);
			else
				SetVelocity(250);
		},
		[this] {
			SetVelocity(0);
		}
	);

}