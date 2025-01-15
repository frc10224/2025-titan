#include "RobotContainer.h"
#include <frc2/command/Commands.h>

Drivetrain *g_drivetrain;
Pose *g_pose;
Shooter *g_shooter;

RobotContainer::RobotContainer() {
    g_drivetrain = &drivetrain;
    g_pose = &pose;
    g_shooter = &shooter;
    // set up controller binds here
}

frc2::CommandPtr RobotContainer::CreateAutoCommand() {
    return frc2::cmd::None();
}

frc2::CommandPtr RobotContainer::CreateTeleopCommand() {
    return drivetrain.MecanumDrive(
        [this] { return this->driver.GetLeftX(); },
        [this] { return this->driver.GetLeftY(); },
        [this] { return this->driver.GetRightX(); });
    return frc2::cmd::None();
}