#include <frc2/command/button/CommandXboxController.h>
#include <frc2/command/CommandPtr.h>

#include "subsystem/Drivetrain.h"
#include "subsystem/Shooter.h"

class RobotContainer {
private:
    frc2::CommandXboxController driver{0};
    Drivetrain drivetrain;
    Shooter shooter;
public:
    RobotContainer();
    frc2::CommandPtr CreateAutoCommand();
    frc2::CommandPtr CreateTeleopCommand();
};