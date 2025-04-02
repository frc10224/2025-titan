package frc.robot;

import org.littletonrobotics.junction.Logger;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.units.measure.Distance;

import static edu.wpi.first.units.Units.*;
import static frc.robot.Constants.PoseConstants.*;

import java.util.Timer;

// this may not be a "subsystem" but it does contain a lot of things that
// should be self contained

public final class Pose {
    private static final Pose instance = new Pose();
    public static Pose getInstance() { return instance; }

    // private construtor (singleton!)
    private Pose() {}

    private PhotonCamera[] cameras = {
        new PhotonCamera("front"),
        //new PhotonCamera("back"),
    };

    private Transform3d robotToTag = null;
    private LinearFilter yawFilter = LinearFilter.movingAverage(5);
    private LinearFilter xFilter = LinearFilter.movingAverage(5);
    private LinearFilter yFilter = LinearFilter.movingAverage(5);
    private int staleMs = 0;

    public Transform3d getRobotToTag() {
        return robotToTag;
    }

    public int getStaleTimeMs() {
    	return staleMs;
    }

    public double getYaw() {
        return yawFilter.lastValue();
    }

    public Distance getTagX() {
        return Meters.of(xFilter.lastValue());
    }

    public Distance getTagY() {
        return Meters.of(yFilter.lastValue());
    }

    private double translationLength(Transform3d t) {
        return Math.sqrt(Math.pow(t.getX(), 2) + Math.pow(t.getY(), 2));
    }

    private double translationLength(Distance x, Distance y) {
        return Math.sqrt(Math.pow(x.baseUnitMagnitude(), 2) + Math.pow(y.baseUnitMagnitude(), 2));
    }

    public void periodicUpdate() {
        PhotonTrackedTarget finalTarget = null;
        double tagDist = 100;

        // pick the closest tag we see
        for (PhotonCamera camera : cameras) {
            for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
                if (!result.hasTargets()) continue;
                for (PhotonTrackedTarget target : result.targets) {
                    double l = translationLength(target.getBestCameraToTarget());
                    if (l < tagDist) {
                        finalTarget = target;
                        tagDist = l;
                    }
                }
            }
        }

        staleMs += 20;
        if (finalTarget != null && tagDist < 100) {
            robotToTag = finalTarget.getBestCameraToTarget().inverse().plus(kFrontCameraLocation);
            Logger.recordOutput("Pose/Tag Distance", translationLength(getTagX(), getTagY()));
            Logger.recordOutput("Pose/Yaw diff from tag", yawFilter.calculate(robotToTag.getRotation().getZ()));
            Logger.recordOutput("Pose/Tag X Distance", xFilter.calculate(robotToTag.getX()));
            Logger.recordOutput("Pose/Tag Y Distance", yFilter.calculate(robotToTag.getY()));
            staleMs = 0;
        }
        Logger.recordOutput("Pose/Last pose ago ms", staleMs);
    }
}

// vi: sw=4 ts=4 noet tw=80 cc=80
