package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.studica.frc.AHRS;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.MecanumDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.MecanumDriveKinematics;
import edu.wpi.first.math.kinematics.MecanumDriveWheelPositions;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

import static frc.robot.Constants.DrivetrainConstants.kBackLeftLocation;
import static frc.robot.Constants.DrivetrainConstants.kBackRightLocation;
import static frc.robot.Constants.DrivetrainConstants.kFrontLeftLocation;
import static frc.robot.Constants.DrivetrainConstants.kFrontRightLocation;
import static frc.robot.Constants.PoseConstants.*;

// this may not be a "subsystem" but it does contain a lot of things that
// should be self contained

public final class Pose {
    private static final Pose instance = new Pose();
    public static Pose getInstance() { return instance; }

    private PhotonCamera[] cameras = {
        new PhotonCamera("front"),
        new PhotonCamera("back"),
    };
    
    private Pose3d visionEstimate = null;
    private AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI); 
    AprilTagFieldLayout tagLayout = 
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    StructPublisher<Pose3d> posePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Pose/estimate", Pose3d.struct).publish();

    private final MecanumDriveKinematics kinematics =
        new MecanumDriveKinematics(kFrontLeftLocation, kFrontRightLocation, kBackLeftLocation, kBackRightLocation);
    // private final MecanumDriveOdometry odometry = new MecanumDriveOdometry(kinematics, navx.getRotation2d(), drivetrain.getWheelPositions());
    private final MecanumDrivePoseEstimator poseEstimator = new MecanumDrivePoseEstimator(
        kinematics,
        navx.getRotation2d(),
        new MecanumDriveWheelPositions(0, 0, 0, 0),
        Pose2d.kZero
    );

    private Pose() {
       
    }

    public void updateWheelPositions(MecanumDriveWheelPositions wheelPositions) {
        poseEstimator.update(navx.getRotation2d(), wheelPositions);
    }

    public void periodicUpdate() {
        PhotonPipelineResult finalResult = null;
        double targetAmbiguity = 1;

        for (PhotonCamera camera : cameras) {
            for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
                PhotonTrackedTarget target = result.getBestTarget();
                if (target != null && tagLayout.getTagPose(target.getFiducialId()).isPresent() && target.getPoseAmbiguity() < targetAmbiguity) {
                    finalResult = result;
                    targetAmbiguity = target.getPoseAmbiguity();
                }
            }
        }

        if (finalResult != null && targetAmbiguity < 1) {
            PhotonTrackedTarget finalTarget = finalResult.getBestTarget();
            visionEstimate = PhotonUtils.estimateFieldToRobotAprilTag(
                    finalTarget.getBestCameraToTarget(),
                    tagLayout.getTagPose(finalTarget.getFiducialId()).get(),
                    kFrontCameraLocation
            );

            double[] stddevMatrix = {targetAmbiguity * kPositionStdev, targetAmbiguity * kPositionStdev, targetAmbiguity * kYawStdev};
            Matrix<N3, N1> matrix = new Matrix<N3, N1>(Nat.N3(), Nat.N1(), stddevMatrix);
            poseEstimator.addVisionMeasurement(visionEstimate.toPose2d(), finalResult.getTimestampSeconds(), matrix);
        }
        
        posePublisher.set(new Pose3d(poseEstimator.getEstimatedPosition()));
    }
}

// vi: sw=4 ts=4 noet tw=80 cc=80