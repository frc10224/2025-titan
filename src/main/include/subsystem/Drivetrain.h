#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/kinematics/MecanumDriveWheelPositions.h>
#include <frc/RobotController.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <rev/SparkMax.h>

#include "Bits.h"
#include "Constants.h"

class Motor {
public:
    rev::spark::SparkMax motor;
    rev::spark::SparkClosedLoopController &pid;
    Motor(int id);
    
    inline units::meter_t GetWheelDistance() {
        return DrivetrainConstants::kWheelRadius *
               DrivetrainConstants::kGearRatio *
               motor.GetEncoder().GetPosition();
    }
    inline void SetVelocity(double percent) {
        pid.SetReference(percent * DrivetrainConstants::kMaxRPM,
            rev::spark::SparkLowLevel::ControlType::kVelocity);
    }
    inline void SetVoltage(units::volt_t voltage) {
        motor.SetVoltage(voltage);
    }
    inline units::turns_per_second_t GetEncoderVelocity() {
        return units::turns_per_second_t{motor.GetEncoder().GetVelocity() / 60};
    }
    void Log(frc::sysid::SysIdRoutineLog *log, const char *title);
};

class Drivetrain : public frc2::SubsystemBase {
private:
    Motor motorLf{DrivetrainConstants::kMotorId_LF};
    Motor motorLb{DrivetrainConstants::kMotorId_LB};
    Motor motorRf{DrivetrainConstants::kMotorId_RF};
    Motor motorRb{DrivetrainConstants::kMotorId_RB}; 

    frc2::sysid::SysIdRoutine sysid_routine;
public:
    Drivetrain();
    void Periodic() override;
    frc2::CommandPtr MecanumDrive(Fn<double> xSpeed, Fn<double> ySpeed, Fn<double> zRotate);
    frc::MecanumDriveWheelPositions GetWheelPositions();
    inline frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction) {
        return sysid_routine.Dynamic(direction);
    }
    inline frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction) {
        return sysid_routine.Quasistatic(direction);
    }
};
