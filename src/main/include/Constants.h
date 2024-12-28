#pragma once

#include <frc/geometry/Translation2d.h>
#include <units/base.h>

namespace DrivetrainConstants {
    const double kMaxDriveSpeed = 1;
    const double kMaxTurnSpeed = 1;
    const int kMotorId_LB = 3;
    const int kMotorId_LF = 4;
    const int kMotorId_RF = 1;
    const int kMotorId_RB = 2;
    const frc::Translation2d kFrontLeftLocation{0.381_m, 0.381_m};
    const frc::Translation2d kFrontRightLocation{0.381_m, -0.381_m};
    const frc::Translation2d kBackLeftLocation{-0.381_m, 0.381_m};
    const frc::Translation2d kBackRightLocation{-0.381_m, -0.381_m};
    const units::meter_t kWheelRadius = 0.20_m;
};

namespace ShooterConstants {
    const int kMotorId_left = 5;
    const int kMotorId_right = 6;
};