#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/kinematics/MecanumDriveWheelPositions.h>
#include <rev/SparkMax.h>

#include "Bits.h"
#include "Constants.h"

class Drivetrain : public frc2::SubsystemBase {
private:
    rev::spark::SparkMax motorLf{DrivetrainConstants::kMotorId_LF, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkMax motorLb{DrivetrainConstants::kMotorId_LB, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkMax motorRf{DrivetrainConstants::kMotorId_RF, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkMax motorRb{DrivetrainConstants::kMotorId_RB, rev::spark::SparkMax::MotorType::kBrushless};
public:
    Drivetrain();
    void Periodic() override;
    frc2::CommandPtr MecanumDrive(Fn<double> xSpeed, Fn<double> ySpeed, Fn<double> zRotate);
    frc::MecanumDriveWheelPositions GetWheelPositions();
};
