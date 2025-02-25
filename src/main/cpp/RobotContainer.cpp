#include "RobotContainer.h"
#include <frc2/command/Commands.h>
#include <cmath>

Drivetrain *g_drivetrain;
Pose *g_pose;

RobotContainer::RobotContainer() {
    g_drivetrain = &drivetrain;
    g_pose = &pose;
    driver.A().WhileTrue(coral.Collect());
    driver.B().WhileTrue(coral.Spit());
    //driver.X().WhileTrue(elevator.SysIdQuasistatic(frc2::sysid::Direction::kReverse   ));
    //driver.Y().WhileTrue(elevator.SysIdQuasistatic(frc2::sysid::Direction::kForward));
    //driver.A().WhileTrue(elevator.SysIdDynamic(frc2::sysid::Direction::kReverse));
    //driver.B().WhileTrue(elevator.SysIdDynamic(frc2::sysid::Direction::kForward));
    driver.X().OnTrue(elevator.MoveUp(1));
    driver.Y().OnTrue(elevator.MoveUp(-1));
}

frc2::CommandPtr RobotContainer::CreateAutoCommand() {
    return frc2::cmd::None();
}

frc2::CommandPtr RobotContainer::CreateTeleopCommand() {
    return drivetrain.MecanumDrive(
        [this] { return std::pow(-this->driver.GetLeftY(), 3.); }, // cube these so it's easier to control
        [this] { return std::pow(this->driver.GetLeftX(), 3.); },
        [this] { return std::pow(this->driver.GetRightX(), 3.); });
    //return frc2::cmd::Run([this] { this->elevator.leftMotor.Set(this->driver.GetLeftX()); });
    //return frc2::cmd::None();
}