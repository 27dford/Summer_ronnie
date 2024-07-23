package frc;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import frc.robot.Swerve;

public class Drivetrain {
    Pigeon2 pigeon = new Pigeon2(0, "*");
    Swerve frontR = new Swerve(2, 2, 6, false, false);
    Swerve backR = new Swerve(0, 6, 4, true, false);
    Swerve frontL = new Swerve(3, 1, 7, false, false);
    Swerve backL = new Swerve(1, 5, 5, false, true);
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


    public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
        new Translation2d(wheelbaseMeters / 2.0, trackwidthMeters / 2.0),
        new Translation2d(wheelbaseMeters / 2.0, -trackwidthMeters / 2.0),
        new Translation2d(-wheelbaseMeters / 2.0, trackwidthMeters / 2.0),
        new Translation2d(-wheelbaseMeters / 2.0, -trackwidthMeters / 2.0)
    );

    public void feildOrienteDrive(ChassisSpeeds desiredFieldSpeed) {
        ChassisSpeeds robotOrientedSpeed = ChassisSpeeds.fromFieldRelativeSpeeds(desiredFieldSpeed, pigeon.getRotation2d());
        robotOrientedDrive(robotOrientedSpeed);

    }
    public void robotOrientedDrive(ChassisSpeeds desiredSpeed) {
        SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(desiredSpeed);
        frontL.drive(moduleStates[0]);
        frontR.drive(moduleStates[1]);
        backL.drive(moduleStates[2]);
        backR.drive(moduleStates[3]);
    }
    public void zeroPigeon() {
        pigeon.setYaw(0);
    }
}
