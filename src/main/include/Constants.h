#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Transform3d.h>
#include <units/length.h>
#include <units/base.h>

namespace DrivetrainConstants {
    const double kMaxDriveSpeed = 1;
    const double kMaxTurnSpeed = 1;
    const double kGearRatio = 1/5.71;
    const units::meter_t kWheelRadius = 2_in;
    const double kP = 0.0000026155;
    const double kI = 0;
    const double kD = 0;
    const double kVelocityFF = 0.11911;
    // Max power RPM
    const double kMaxRPM = 2900;
    // CAN bus IDs
    const int kMotorId_LB = 2;
    const int kMotorId_LF = 1;
    const int kMotorId_RF = 3;
    const int kMotorId_RB = 4;
    const frc::Translation2d kFrontLeftLocation{0.381_m, 11_in};
    const frc::Translation2d kFrontRightLocation{0.381_m, -11_in};
    const frc::Translation2d kBackLeftLocation{-0.381_m, 0.381_m};
    const frc::Translation2d kBackRightLocation{-0.381_m, -0.381_m};
};

namespace PoseConstants {
    // TODO: fix these!
    const frc::Transform3d kBackCameraLocation{
        frc::Translation3d(0_m, 0_m, 0_m), frc::Rotation3d(0_rad, 0_rad, 0_rad)};
}

namespace ElevatorConstants {
    const int kLeftMotorId = 6;
    const int kRightMotorId = 5;
    const double kP = 0.086188;
    const double kD = 0.92;
    const double kFF = 0.12367;
    const double kTopLimitSpinCount = 89;
}

namespace CoralConstants {
    const int kLeftMotorId = 7;
    const int kRightMotorId = 8;
    const double kP = 0.0001;
    const double kD = 0;
    const double kFF = 0.0003;
    const int kLaserCanId = 9;
}
