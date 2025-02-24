#include <frc2/command/SubsystemBase.h>
#include <frc2/command/sysid/SysIdRoutine.h>
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
    std::optional<grpl::LaserCanMeasurement> laser_measurement;
    frc2::sysid::SysIdRoutine sysid_routine;
    void SetVelocity(double rpm);
public:
    Coral();
    void Periodic() override;
    frc2::CommandPtr Collect();
    frc2::CommandPtr Spit();
    inline frc2::CommandPtr SysIdDynamic(frc2::sysid::Direction direction) {
        return sysid_routine.Dynamic(direction);
    }
    inline frc2::CommandPtr SysIdQuasistatic(frc2::sysid::Direction direction) {
        return sysid_routine.Quasistatic(direction);
    }
};