#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>

#include "Constants.h"

class Elevator : public frc2::SubsystemBase {
private:
    rev::spark::SparkMax leftMotor{ElevatorConstants::kLeftMotorId,
        rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkMax rightMotor{ElevatorConstants::kRightMotorId,
        rev::spark::SparkMax::MotorType::kBrushless};
public:
    Elevator();
    void Periodic() override;
    void SetPosition(double turns);
};