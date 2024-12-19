#include <frc2/command/SubsystemBase.h>
#include <frc/drive/MecanumDrive.h>
#include <rev/SparkMax.h>

#include "Bits.h"
#include "Constants.h"

class Drivetrain : public frc2::SubsystemBase {
private:
    rev::spark::SparkMax motorLf{MOTORID_FL, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkMax motorLb{MOTORID_RL, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkMax motorRf{MOTORID_FR, rev::spark::SparkMax::MotorType::kBrushless};
    rev::spark::SparkMax motorRb{MOTORID_RR, rev::spark::SparkMax::MotorType::kBrushless};

public:
    frc::MecanumDrive m_robotDrive{
        [&](double output) { motorLf.Set(output); },
        [&](double output) { motorLb.Set(output); },
        [&](double output) { motorRf.Set(output); },
        [&](double output) { motorRb.Set(output); }};
public:
    Drivetrain();
    frc2::CommandPtr MecanumDrive(Fn<double> xSpeed, Fn<double> ySpeed, Fn<double> zRotate);
    void Periodic() override;
};
