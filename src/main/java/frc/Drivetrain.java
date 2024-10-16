package frc;

import java.util.Optional;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Camera;
import frc.robot.PoseWithTimestamp;
import frc.robot.Swerve;

public class Drivetrain {
    Pigeon2 pigeon = new Pigeon2(0, "*");
    Swerve frontR = new Swerve(2, 2, 6, false, false);
    Swerve backR = new Swerve(0, 6, 4, true, false);
    Swerve frontL = new Swerve(3, 1, 7, false, false);
    Swerve backL = new Swerve(1, 5, 5, false, true);
    double rotate = 0;
    Camera shooterCamera2 = new Camera();

    Field2d field = new Field2d();

    private SwerveDrivePoseEstimator fusedPoseEstimator;
    /**
     * Distance between the center point of the left wheels and the center point of the right wheels.
     */
    public static final double trackwidthMeters = Units.inchesToMeters(23.75);
    /**
     * Distance between the center point of the front wheels and the center point of the back wheels.
     */
    public static final double wheelbaseMeters = Units.inchesToMeters(22.75);
    /**
     * Distance from the center of the robot to each swerve module.
     */
    public static final double drivetrainRadiusMeters = Math.hypot(wheelbaseMeters / 2.0, trackwidthMeters / 2.0); //0.4177

    public Drivetrain() {
        fusedPoseEstimator = new SwerveDrivePoseEstimator(
            kinematics, 
            pigeon.getRotation2d(),
            getModulePositions(),
            new Pose2d()
        );

        SmartDashboard.putData(field);
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] output = new SwerveModulePosition[4];
        output[0] = frontL.getPosition();
        output[1] = frontR.getPosition();
        output[2] = backL.getPosition();
        output[3] = backR.getPosition();
        return output;
    }


    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(wheelbaseMeters / 2.0, trackwidthMeters / 2.0),
        new Translation2d(wheelbaseMeters / 2.0, -trackwidthMeters / 2.0),
        new Translation2d(-wheelbaseMeters / 2.0, trackwidthMeters / 2.0),
        new Translation2d(-wheelbaseMeters / 2.0, -trackwidthMeters / 2.0)
    );

    public void feildOrienteDrive(ChassisSpeeds desiredFieldSpeed) {
        ChassisSpeeds robotOrientedSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(desiredFieldSpeed, fusedPoseEstimator.getEstimatedPosition().getRotation());
        robotOrientedDrive(robotOrientedSpeed);

    }
    public void robotOrientedDrive(ChassisSpeeds desiredSpeed) {
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(desiredSpeed);
        frontL.drive(moduleStates[0]);
        frontR.drive(moduleStates[1]);
        backL.drive(moduleStates[2]);
        backR.drive(moduleStates[3]);
    }
    public void settingPigeon(double yawDegrees) {
        // pigeon.setYaw(yawDegrees);
    }
    public void zeroGyro() {
        Pose2d currentPose = fusedPoseEstimator.getEstimatedPosition();
        fusedPoseEstimator.resetPosition(pigeon.getRotation2d(), getModulePositions(), new Pose2d(currentPose.getTranslation(), new Rotation2d()));
    }
    public boolean turntoAngle(double angleDeg) {
        rotate =  angleDeg - fusedPoseEstimator.getEstimatedPosition().getRotation().getDegrees();
        if ((rotate > 2) || (rotate < -2)) {
            rotate =  angleDeg - fusedPoseEstimator.getEstimatedPosition().getRotation().getDegrees();
            double desiredDegreesPerSecond = 6 * rotate;
            ChassisSpeeds speed = new ChassisSpeeds(0,0,Units.degreesToRadians(desiredDegreesPerSecond));
            feildOrienteDrive(speed);
            return false;
        }
        else {
            ChassisSpeeds speed = new ChassisSpeeds(0,0,0);
            feildOrienteDrive(speed);
            return true;
        }
    }

    public void updatePoseEstimator() {
        fusedPoseEstimator.update(pigeon.getRotation2d(), getModulePositions());

        Optional<PoseWithTimestamp> poseFromCamera = shooterCamera2.getRobotPose();
        if (poseFromCamera.isEmpty()) {
            return;
        }

        fusedPoseEstimator.addVisionMeasurement(poseFromCamera.get().pose, poseFromCamera.get().timestamp);

        field.setRobotPose(fusedPoseEstimator.getEstimatedPosition());
    }

    public Pose2d getPose() {
        return fusedPoseEstimator.getEstimatedPosition();
    }
}
