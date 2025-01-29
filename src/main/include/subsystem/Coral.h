#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>

#include "Constants.h"
#include "Bits.h"

class Coral : public frc2::SubsystemBase {
private:
    rev::spark::SparkMax leftMotor{CoralConstants::kLeftMotorId,
        rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkMax rightMotor{CoralConstants::kRightMotorId,
        rev::spark::SparkMax::MotorType::kBrushless};
public:
    Coral();
    void Periodic() override;
    frc2::CommandPtr Feed(Fn<double> Speed);
};