#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
#include <rev/SparkMax.h>

#include "Constants.h"

class Elevator : public frc2::SubsystemBase {
public:
    rev::spark::SparkMax leftMotor{ElevatorConstants::kLeftMotorId,
        rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkMax rightMotor{ElevatorConstants::kRightMotorId,
        rev::spark::SparkMax::MotorType::kBrushless};
    frc2::sysid::SysIdRoutine sysid_routine;
    double setpoint;
public:
    Elevator();
    void Periodic() override;
    frc2::CommandPtr SetPosition(double turns);
    frc2::CommandPtr MoveUp(double turns);
    inline frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction) {
        return sysid_routine.Dynamic(direction);
    }
    inline frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction) {
        return sysid_routine.Quasistatic(direction);
    }
};