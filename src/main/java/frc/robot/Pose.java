package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.studica.frc.AHRS;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;

import static frc.robot.Constants.PoseConstants.*;

// this may not be a "subsystem" but it does contain a lot of things that
// should be self contained

public final class Pose {
    private static final Pose instance = new Pose();
    public static Pose getInstance() { return instance; }

    PhotonCamera frontCamera = new PhotonCamera("front");
    PhotonCamera backCamera = new PhotonCamera("back");
    
    Pose3d poseEstimate = null;
    AHRS navx = new AHRS(AHRS.NavXComType.kMXP_SPI); 
    AprilTagFieldLayout tagLayout = 
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    StructPublisher<Pose3d> posePublisher = NetworkTableInstance.getDefault()
        .getStructTopic("Pose/visionEstimate", Pose3d.struct).publish();

    private Pose() {}

    public void periodicUpdate() {
        for (PhotonPipelineResult result : frontCamera.getAllUnreadResults()) {
            // Calculate robot's field relative pose
            PhotonTrackedTarget target = result.getBestTarget();
            if (target != null && tagLayout.getTagPose(target.getFiducialId()).isPresent()) {
                poseEstimate = PhotonUtils.estimateFieldToRobotAprilTag(
                    target.getBestCameraToTarget(),
                    tagLayout.getTagPose(target.getFiducialId()).get(),
                    kFrontCameraLocation);
                posePublisher.set(poseEstimate);
            }
        }
    }
}

// vi: sw=4 ts=4 noet tw=80 cc=80