#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>

#include "subsystem/Shooter.h"
#include "Constants.h"
#include "Bits.h"

void Shooter::Periodic() {
	// put smartdash infos here etc
}

frc2::CommandPtr Shooter::SpinMotors(Fn<double> GetSpeed) {
	return frc2::cmd::RunOnce([GetSpeed, this] {
		double speed = GetSpeed();
		this->left.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, speed);
		this->right.Set(ctre::phoenix::motorcontrol::ControlMode::PercentOutput, -speed);
	});
}