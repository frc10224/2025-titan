#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystem/LimelightHelpers.h"

#include "subsystem/Pose.h"
#include "Bits.h"

Pose::Pose() : 
        aprilTagLayout(frc::AprilTagFieldLayout::LoadField(
            frc::AprilTagField::k2025ReefscapeAndyMark)) {
    navx.Reset();
    backVisionEstimator.SetMultiTagFallbackStrategy(photon::LOWEST_AMBIGUITY);
}

void Pose::Periodic() {
    double yaw = navx.GetYaw();
    frc::SmartDashboard::PutNumber("Robot yaw", yaw);

    // ignore vision measurements if we are spinning they are probably stale
    if (navx.GetRate() > 720) return;

    // Update from PhotonVision coprocessor
    std::vector<photon::PhotonPipelineResult> results =
        backCamera.GetAllUnreadResults();
    for (auto result : results) {
        if (std::optional<photon::EstimatedRobotPose> estimate =
                backVisionEstimator.Update(result)) {
            driveEstimator.AddVisionMeasurement(
                estimate->estimatedPose.ToPose2d(), estimate->timestamp);
        }
    }

    // update from the Limelight
    LimelightHelpers::SetRobotOrientation("", yaw, 0, 0, 0, 0, 0);
    LimelightHelpers::PoseEstimate estimate =
        LimelightHelpers::getBotPoseEstimate_wpiBlue_MegaTag2();
    if (estimate.tagCount > 0)
        driveEstimator.AddVisionMeasurement(estimate.pose, estimate.timestampSeconds);
}

void Pose::UpdateFromWheelPositions(frc::MecanumDriveWheelPositions wheelPos) {
    pose = driveEstimator.Update(
        frc::Rotation2d(units::degree_t{navx.GetYaw()}), wheelPos);
}