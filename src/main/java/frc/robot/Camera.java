package frc.robot;
import java.util.List;
import java.util.Optional;

import javax.swing.text.html.Option;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class Camera {
    double returnDistance = 0;
    PhotonCamera camera = new PhotonCamera("shooterCamera");
    public final static Transform3d robotToShooterCamera = new Transform3d(
        new Translation3d(Units.inchesToMeters(9.25), 0, Units.inchesToMeters(10.625)),
        new Rotation3d(0, Math.toRadians(-28), 0)
    );
    PhotonPoseEstimator poseEstimator = new PhotonPoseEstimator(
        AprilTagFields.k2024Crescendo.loadAprilTagLayoutField(), 
        PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        camera,
        robotToShooterCamera
    );

    public double hasCorrectTarget() {
        var result = camera.getLatestResult();
        List<PhotonTrackedTarget> targets = result.getTargets();
        for (int i = 0; i < targets.size(); i++){
            if(targets.get(0).getFiducialId() == 4) {
                return(i);
            }  
        }
        return 111;
    }
    
    public double getDistance() {
        var result = camera.getLatestResult();
        List<PhotonTrackedTarget> targets = result.getTargets();
        double correctTarget = hasCorrectTarget();
        if (correctTarget == 111) {
            return 0;
        } else if(correctTarget == 1) {
            return(result.targets.get(1).getBestCameraToTarget().getTranslation().getNorm());
        } else {
            return(result.targets.get(0).getBestCameraToTarget().getTranslation().getNorm());
        }
    }
    public double getYaw() {
        var result = camera.getLatestResult();
        List<PhotonTrackedTarget> targets = result.getTargets();
        double correctTarget = hasCorrectTarget();
        if (correctTarget == 111) {
            return 0;
        } else if(correctTarget == 1) {
            return(result.targets.get(1).getYaw());
        } else {
            return(result.targets.get(0).getYaw());
        }

    }

    public Optional<PoseWithTimestamp> getRobotPose() {
        Optional<EstimatedRobotPose> poseEstimatorResult = poseEstimator.update();
        if (poseEstimatorResult.isEmpty()) {
            return Optional.empty();
        }

        EstimatedRobotPose poseEstimate = poseEstimatorResult.get();
        return Optional.of(new PoseWithTimestamp(poseEstimate.estimatedPose.toPose2d(), poseEstimate.timestampSeconds));
    }
}