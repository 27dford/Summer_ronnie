package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;

public class PoseWithTimestamp {
    public double timestamp;
    public Pose2d pose;

    public PoseWithTimestamp(Pose2d pose, double timestamp) {
        this.pose = pose;
        this.timestamp = timestamp;
    }
}
