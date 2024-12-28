#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystem/Pose.h"
#include "Bits.h"

Pose::Pose() {
    frc::SmartDashboard::PutData("Pose", &field);
    LimelightHelpers::setLEDMode_PipelineControl();
    navx.Reset();
}

void Pose::Periodic() {
    LimelightHelpers::SetRobotOrientation("", navx.GetYaw(), 0, 0, 0, 0, 0);
    LimelightHelpers::PoseEstimate estimate =
        LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2();

    // if we are spinning super fast just ignore the vision measurement since
    // they are probably stale
    if (estimate.tagCount && navx.GetRate() <= 720)
        estimator.AddVisionMeasurement(estimate.pose, estimate.timestampSeconds);

    field.SetRobotPose(pose);
}

void Pose::UpdateFromWheelPositions(frc::MecanumDriveWheelPositions wheelPos) {
    pose = estimator.Update(frc::Rotation2d(units::degree_t{navx.GetYaw()}), wheelPos);
}