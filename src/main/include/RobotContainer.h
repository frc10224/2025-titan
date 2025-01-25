#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/CommandPtr.h>

#include "subsystem/Drivetrain.h"
#include "subsystem/Coral.h"
#include "subsystem/Elevator.h"
#include "subsystem/Pose.h"

class RobotContainer {
private:
    frc2::CommandXboxController driver{0};
    Drivetrain drivetrain;
    Elevator elevator;
    Pose pose;
public:
    RobotContainer();
    frc2::CommandPtr CreateAutoCommand();
    frc2::CommandPtr CreateTeleopCommand();
};