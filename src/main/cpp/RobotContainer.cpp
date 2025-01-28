#include "RobotContainer.h"
#include <frc2/command/Commands.h>
#include <cmath>

Drivetrain *g_drivetrain;
Pose *g_pose;

RobotContainer::RobotContainer() {
    g_drivetrain = &drivetrain;
    g_pose = &pose;
    // set up controller binds here
}

frc2::CommandPtr RobotContainer::CreateAutoCommand() {
    return frc2::cmd::None();
}

frc2::CommandPtr RobotContainer::CreateTeleopCommand() {
    return drivetrain.MecanumDrive(
        [this] { return std::pow(this->driver.GetLeftX(), 3.); }, // cube these so it's easier to control
        [this] { return std::pow(this->driver.GetLeftY(), 3.); },
        [this] { return std::pow(this->driver.GetRightX(), 3.); });
}