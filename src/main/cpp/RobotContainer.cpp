#include "RobotContainer.h"
#include <frc2/command/Commands.h>

RobotContainer::RobotContainer() {
    driver.RightTrigger().OnTrue(shooter.SpinMotors([] { return 1.0; }));
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
}