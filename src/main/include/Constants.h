#pragma once

#include <frc/geometry/Translation2d.h>
#include <frc/geometry/Transform3d.h>
#include <units/base.h>

namespace DrivetrainConstants {
    // TODO: fix these!
    const double kMaxDriveSpeed = 1;
    const double kMaxTurnSpeed = 1;
    const double kGearRatio = 5.71;
    const units::inch_t kWheelRadius = 2_in;
    const double kP = 0.001;
    const double kI = 0;
    const double kD = 0.001;
    // https://docs.revrobotics.com/brushless/neo/v1.1#motor-specifications
    // This value should be 1/Kv
    const double kVelocityFF = 1/473;
    // Max power RPM
    const double kMaxRPM = 2900;
    // CAN bus IDs
    const int kMotorId_LB = 3;
    const int kMotorId_LF = 4;
    const int kMotorId_RF = 1;
    const int kMotorId_RB = 2;
    const frc::Translation2d kFrontLeftLocation{0.381_m, 0.381_m};
    const frc::Translation2d kFrontRightLocation{0.381_m, -0.381_m};
    const frc::Translation2d kBackLeftLocation{-0.381_m, 0.381_m};
    const frc::Translation2d kBackRightLocation{-0.381_m, -0.381_m};
};

namespace PoseConstants {
    // TODO: fix these!
    const frc::Transform3d kBackCameraLocation{
        frc::Translation3d(0_m, 0_m, 0_m), frc::Rotation3d(0_rad, 0_rad, 0_rad)};
}

namespace ElevatorConstants {
    // TODO: get real ids
    const int kLeftMotorId = 5;
    const int kRightMotorId = 6;
    const double kP = 1;
    const double kD = 1;
    const double kFF = 1;
}

namespace CoralConstants {
    const int kLeftMotorId = 9;
    const int kRightMotorId = 10;
    const int kP = 0;
    const int kD = 0;
}
