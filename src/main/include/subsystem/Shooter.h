#pragma once

#include <frc2/command/SubsystemBase.h>
#include <ctre/phoenix/motorcontrol/can/TalonSRX.h>

#include "Bits.h"
#include "Constants.h"


class Shooter : public frc2::SubsystemBase {
public:
    ctre::phoenix::motorcontrol::can::TalonSRX left{ShooterConstants::kMotorId_left};
    ctre::phoenix::motorcontrol::can::TalonSRX right{ShooterConstants::kMotorId_right};

public:
    frc2::CommandPtr SpinMotors(Fn<double> speed);
    void Periodic() override;
};
