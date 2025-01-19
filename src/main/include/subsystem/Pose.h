#pragma once

#include <frc/apriltag/AprilTagFieldLayout.h>
#include <frc/drive/MecanumDrive.h>
#include <frc/estimator/MecanumDrivePoseEstimator.h>
#include <frc/kinematics/MecanumDriveWheelPositions.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/SubsystemBase.h>

#include <photon/PhotonCamera.h>
#include <photon/PhotonPoseEstimator.h>
#include <studica/AHRS.h>

#include "Drivetrain.h"

extern Drivetrain *g_drivetrain;

class Pose : frc2::SubsystemBase {
private:
    studica::AHRS navx{studica::AHRS::NavXComType::kMXP_SPI};

    frc::AprilTagFieldLayout aprilTagLayout;
    photon::PhotonCamera backCamera{"backCamera"};
    photon::PhotonPoseEstimator backVisionEstimator{
        aprilTagLayout,
        photon::MULTI_TAG_PNP_ON_COPROCESSOR,
        PoseConstants::kBackCameraLocation,
    };
    frc::Pose2d pose;
    frc::MecanumDriveKinematics kinematics{
        DrivetrainConstants::kFrontLeftLocation,
        DrivetrainConstants::kFrontRightLocation,
        DrivetrainConstants::kBackLeftLocation,
        DrivetrainConstants::kBackRightLocation,
    };
    frc::MecanumDrivePoseEstimator driveEstimator{kinematics,
        GyroAngle(), frc::MecanumDriveWheelPositions{}, pose};
public:
    Pose();
    void Periodic() override;
    void UpdateFromWheelPositions(frc::MecanumDriveWheelPositions wheelPos);
    inline frc::Rotation2d GyroAngle() {
        return frc::Rotation2d((units::degree_t)navx.GetYaw());
    }
    inline void ResetAngle() { navx.ZeroYaw(); }
};