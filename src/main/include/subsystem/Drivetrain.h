#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/kinematics/MecanumDriveWheelPositions.h>
#include <rev/SparkMax.h>

#include "Bits.h"
#include "Constants.h"

class Motor {
public:
    rev::spark::SparkMax motor;
    rev::spark::SparkClosedLoopController &pid;
    Motor(int id);
    
    inline units::meter_t GetWheelDistance() {
        // the SparkMax encoder returns motor revolutions, multiply to get to
        // the distance covered by the wheel
        return DrivetrainConstants::kWheelRadius *
               DrivetrainConstants::kGearRatio *
               motor.GetEncoder().GetPosition();
    }

    inline void SetVelocity(double percent) {
        pid.SetReference(percent * DrivetrainConstants::kMaxRPM,
            rev::spark::SparkLowLevel::ControlType::kVelocity);
    }
};

class Drivetrain : public frc2::SubsystemBase {
private:
    Motor motorLf{DrivetrainConstants::kMotorId_LF};
    Motor motorLb{DrivetrainConstants::kMotorId_LB};
    Motor motorRf{DrivetrainConstants::kMotorId_RF};
    Motor motorRb{DrivetrainConstants::kMotorId_RB};
public:
    Drivetrain();
    void Periodic() override;
    frc2::CommandPtr MecanumDrive(Fn<double> xSpeed, Fn<double> ySpeed, Fn<double> zRotate);
    frc::MecanumDriveWheelPositions GetWheelPositions();
};
