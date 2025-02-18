#include <frc2/command/SubsystemBase.h>
#include <grpl/LaserCan.h>
#include <rev/SparkMax.h>

#include "Constants.h"
#include "Bits.h"

class Coral : public frc2::SubsystemBase {
private:
    rev::spark::SparkMax leftMotor{CoralConstants::kLeftMotorId,
        rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkMax rightMotor{CoralConstants::kRightMotorId,
        rev::spark::SparkMax::MotorType::kBrushless};
    grpl::LaserCan laser{CoralConstants::kLaserCanId};
public:
    Coral();
    void Periodic() override;
    frc2::CommandPtr Collect();
    frc2::CommandPtr Spit();
};