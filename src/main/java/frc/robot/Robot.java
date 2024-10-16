// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
// 21 inches left to right
// 20.5 front to back
package frc.robot;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.Drivetrain;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
   int positions = 0;
   Timer timer = new Timer();
   XboxController xbox = new XboxController(0);
   Drivetrain driveTrain = new Drivetrain();
   Intake intake = new Intake();
   Shooter shooter = new Shooter();
   Arm arm = new Arm();
   Camera shooterCamera = new Camera();
   Leds leds = new Leds();
   double storingAngle = 0;
   double storingSpeed = 0;
   boolean ringRGB = true;
   double distance = 0;
   double yaw = 0;
   boolean rotationComplete;
   boolean shooting = false;
   Timer shootTimer = new Timer();
  public void robotInit()
  {
    timer.start();
  }

  /**
   * @param desiredArmAngle degrees
   * @param desiredDrivetrainAngle degrees
   */
  public void autoAim(double desiredArmAngle, double desiredDrivetrainAngle) {
    boolean armAngleDone = arm.getToAngle(desiredArmAngle);
    boolean turnDone = driveTrain.turntoAngle(desiredDrivetrainAngle);
    double leftFlywheelMetersPerSecond = 25;
    double rightFlywheelMetersPerSecond = 20;
    boolean gotToSpeed = shooter.setSpeed(leftFlywheelMetersPerSecond, rightFlywheelMetersPerSecond);
    boolean readyToShoot = armAngleDone && turnDone && gotToSpeed;

    SmartDashboard.putBoolean("armAngleDone", armAngleDone);
    SmartDashboard.putBoolean("turnDone", turnDone);
    SmartDashboard.putBoolean("gotToSpeed", gotToSpeed);

    if (readyToShoot) {
      shootTimer.start();
    }

    if (shootTimer.get() < 1.5 && shootTimer.get() > 0) {
      intake.shoot();
    }
    else {
      intake.stopMotor();
    }
  }

  public double getDriveAngleToSpeaker() {
    Translation2d ronnieLocation = driveTrain.getPose().getTranslation();
    Translation2d speakerLocation = getSpeakerPose().getTranslation().toTranslation2d();

    Translation2d ronnieToSpeaker = speakerLocation.minus(ronnieLocation);
    double angleRadians = Math.atan2(ronnieToSpeaker.getY(), ronnieToSpeaker.getX());
    return Units.radiansToDegrees(angleRadians);
  }

  public Pose3d getSpeakerPose() {
      double speakerUpperLipHeightMeters = Units.inchesToMeters(82.90);
      double speakerLowerLipHeightMeters = Units.inchesToMeters(78.13);
      double speakerDepthIntoFieldMeters = Units.inchesToMeters(18.11);
      double speakerHeightMeters = (speakerUpperLipHeightMeters + speakerLowerLipHeightMeters) / 2.;

      double speakerY = Units.inchesToMeters(218.42);
      double redSpeakerX = Units.inchesToMeters(652.73-1.5) - (speakerDepthIntoFieldMeters / 2.);
      double blueSpeakerX = 0 + (speakerDepthIntoFieldMeters / 2.);

      Translation3d redLocation = new Translation3d(redSpeakerX, speakerY, speakerHeightMeters);
      Rotation3d redOrientation = new Rotation3d(0 , 0, Math.toRadians(180));
      Translation3d blueLocation = new Translation3d(blueSpeakerX, speakerY, speakerHeightMeters);
      Rotation3d blueOrientation = new Rotation3d(0, 0, 0);

      Pose3d redPose = new Pose3d(redLocation, redOrientation);
      Pose3d bluePose = new Pose3d(blueLocation, blueOrientation);
      return redPose;
  }

  @Override
  public void robotPeriodic() {
    driveTrain.updatePoseEstimator();
  }

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic()
  {
    distance = shooterCamera.getDistance();
    yaw = shooterCamera.getYaw();
    SmartDashboard.putNumber("distance", distance);
    double x = -xbox.getLeftY();
    double y = -xbox.getLeftX();
    double x2 = -xbox.getRightX();
    boolean a = xbox.getAButton();
    boolean b = xbox.getBButton();
    boolean leftShoulder = xbox.getLeftBumper();
    boolean rightShoulder = xbox.getRightBumper();
    boolean yXbox = xbox.getYButton();
    double leftTrigger = xbox.getLeftTriggerAxis();
    double rightTrigger = xbox.getRightTriggerAxis();

    if (rightShoulder) {
      double leftFlywheelMetersPerSecond = 25;
      double rightFlywheelMetersPerSecond = 20;
      double estimatedExitVelocity = (leftFlywheelMetersPerSecond + rightFlywheelMetersPerSecond) / 2.;
      double armRadians = getGravCompensatedArmDesiredRadians(getSpeakerPose().getTranslation(), estimatedExitVelocity, false);
      double armRotations = Units.radiansToRotations(armRadians);
      autoAim(armRotations, getDriveAngleToSpeaker());
    } else {
      shootTimer.stop();
      shootTimer.reset();
    if (xbox.getPOV() == 0) {
      driveTrain.zeroGyro();
    }
    x2 = x2 * 6;
    x = x * 2;
    y = y * 2;
    double angleRadians = Math.atan2(y, x);
    if(Math.sqrt(x*x + y*y) < 0.2) {
      y = 0;
      x = 0;
    }
    if(Math.sqrt(x2*x2) < 0.2) { 
      x2 = 0;
    }
    storingAngle = y;
    if (a) {
      arm.getToAngle(Units.radiansToRotations(angleRadians));
    }
    else {
      ChassisSpeeds speed = new ChassisSpeeds(x,y,x2);
      driveTrain.feildOrienteDrive(speed);
      arm.moveArm(0);
    }
    if (b) {
      intake.getRing();
    } else if (!yXbox){
      intake.stopMotor();
    }
    if (intake.hasRing() || !ringRGB) {
      if (ringRGB = true) {
        leds.resetTimer();
      }
      ringRGB = leds.intakedRing();
    } else {
      leds.fadingRainbow();
    }
    if (yXbox) {
      shooter.shoot();
    } else {
      shooter.notShoot();
    }
    if (rightTrigger > 0.1) {
      arm.moveArm(rightTrigger * 1.5);
    } else if(leftTrigger > 0.1) {
      arm.moveArm(-leftTrigger * 1.5);
    } else {
      //arm.lockArm();
    }
    if (leftShoulder) {
      intake.shoot();
    } else if(!b){
      // intake.stopMotor();
    }
  }
  }
  @Override
  public void disabledInit() {
    timer.reset();
  }



  @Override
  public void disabledPeriodic() {
    if (timer.get() < 10){
    leds.fadingRainbow();
    } else if (timer.get() <= 20) {
      leds.fireLED();
    } else if (timer.get() <= 30) {
      leds.rainbowHSV();
    } else if (timer.get() <= 40) {
      leds.aLED();
    } else if (timer.get() <= 50) {
      leds.zoomingHSV();
    } else if (timer.get() >= 50) {
      leds.lowHighHSV();
    }
  }

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}

  /**
     * Gets the angle that the shooter needs to aim at in order for a note to hit the target
     * This accounts for distance and gravity.
     * @return - Angle in degrees, with 0 being straight forward and a positive angle being pointed upwards.
  */
  private double getGravCompensatedArmDesiredRadians(Translation3d targetLocation, double exitVelocityMetersPerSecond, boolean isLobShot) {
      //see https://www.desmos.com/calculator/jhuanigvbs
      // 9.81
      double g = 15;
      double verticalDistance = getVerticalMetersToPivot(targetLocation);
      double horizontalDistance = getHorizontalMetersToPivot(targetLocation);
      double armAngleRadians = 0;

      // TODO: documentation for taking arm length into account
      for (int approximationCount = 0; approximationCount < 4; approximationCount += 1) {
          double a = (g*g)/4.;
          double b = (verticalDistance * g) - (exitVelocityMetersPerSecond * exitVelocityMetersPerSecond);
          double c = (horizontalDistance * horizontalDistance) + (verticalDistance * verticalDistance);

          double tShort = Math.sqrt((-b - Math.sqrt((b*b) - (4*a*c)))/(2*a));
          double tLong =  Math.sqrt((-b + Math.sqrt((b*b) - (4*a*c)))/(2*a));
          double timeToImpact = tShort; // seconds
          if (isLobShot) {
              timeToImpact = tLong;
          }

          double horizontalVelocity = horizontalDistance / timeToImpact;
          double verticalVelocity = (verticalDistance / timeToImpact) + (0.5*g*timeToImpact);
          armAngleRadians = Math.atan2(verticalVelocity, horizontalVelocity);

          // update for next iteration.
          // TODO: should I take the note length into account for considering the distance it travels when in free fall?
          verticalDistance = getVerticalMetersToFlywheels(targetLocation, armAngleRadians);
          horizontalDistance = getHorizontalMetersToFlywheels(targetLocation, armAngleRadians);
      }
      return armAngleRadians;
  }

private double getVerticalMetersToPivot(Translation3d targetLocation) {
  double pivotHeightMeters = Units.inchesToMeters(23.5);
    return targetLocation.getZ() - pivotHeightMeters;
}

private double getHorizontalMetersToPivot(Translation3d targetLocation) {
  Translation2d robotLocation = driveTrain.getPose().getTranslation();
  double robotToTargetDistance = targetLocation.toTranslation2d().minus(robotLocation).getNorm();
  double pivotOffsetMeters = Units.inchesToMeters(9);
  return robotToTargetDistance + pivotOffsetMeters;
}

private double getVerticalMetersToFlywheels(Translation3d targetLocation, double armAngleRadians) {
  double armLengthMeters = Units.inchesToMeters(18.25);
  return getVerticalMetersToPivot(targetLocation) - (armLengthMeters * Math.sin(armAngleRadians));
}

private double getHorizontalMetersToFlywheels(Translation3d targetLocation, double armAngleRadians) {
  double armLengthMeters = Units.inchesToMeters(18.25);
  return getHorizontalMetersToPivot(targetLocation) - (armLengthMeters * Math.cos(armAngleRadians));
}


}
