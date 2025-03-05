package frc.robot;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static frc.robot.Constants.PoseConstants.*;

// this may not be a "subsystem" but it does contain a lot of things that
// should be self contained

public final class Pose {
    private static final Pose instance = new Pose();
    public static Pose getInstance() { return instance; }

    PhotonCamera frontCamera = new PhotonCamera("front");
    PhotonCamera backCamera = new PhotonCamera("back");
    
    Pose3d poseEstimate;
    AHRS navx = new AHRS(NavXComType.kMXP_SPI);
    AprilTagFieldLayout tagLayout =
        AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

    public void periodicUpdate() {
        for (PhotonPipelineResult result : frontCamera.getAllUnreadResults()) {
            // Calculate robot's field relative pose
            PhotonTrackedTarget target = result.getBestTarget();
            if (tagLayout.getTagPose(target.getFiducialId()).isPresent()) {
                Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                    target.getBestCameraToTarget(),
                    tagLayout.getTagPose(target.getFiducialId()).get(),
                    kFrontCameraLocation);
                //SmartDashboard.putNumberArray("Pose/Estimate", {robotPose.getMeasureX().as(Meters), robotPose.getMeasureY(), robotPose.getMeasureZ()});
            }
        }
    }
}
